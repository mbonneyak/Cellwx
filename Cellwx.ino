#include "LowPower.h"
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include <DHT22.h>
#include <stdio.h>

//Git Test
#define DHT_PIN1 11     // DHT22 #1 Input Pin
#define DHT_PIN2 12     // DHT22 #2 Input Pin
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define FONA_RX_PIN 7      // FONA RX Pin
#define FONA_TX_PIN 6      // FONA TX Pin
#define FONA_RST_PIN 4     // FONA Reset Pin
#define FONA_KEY_PIN 5     // FONA KEY Pin (on/off)
#define FONA_PS_PIN 8      // FONA PS (Power Status) Pin
#define WindSpeed_PIN 3    // Wind Speed Pin
#define WindDir_PIN 2      // Wind Direction Pin
#define V_BAT_PIN A3       // Analog Input Pin Sensing the battery voltage
#define V_SOLAR_PIN A0      // Analog Input Pin Sensing the Solar Charger Output Voltage to Load
#define Debug_PIN 9         // Input to signal Programming is needed
#define LED 13             // Built-in LED

//Objects
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX_PIN, FONA_RX_PIN); // or comment this out & use a hardware serial port like Serial1 (see below)
Adafruit_FONA fona = Adafruit_FONA(FONA_RST_PIN);
DHT22 dht1(DHT_PIN1);

//Global Variable
boolean debugMode = true;
char IMEI[] = "123456789123456";
char getSettingsURL[] = "www.cellwx.org/settings.php?imei=XXXXXXXXXXXXXXX&time=XX";
char sendLogEntryURL[] = "www.cellwx.org/log.php?imei=XXXXXXXXXXXXXXX&messageid=XX";
char sendDataURL[] = "www.cellwx.org/data.php?imei=000000000000000&mws=0000&aws=0000&pws=0000&wd=000&t1=0000&t2=0000&h1=000&h2=000&slp=0000&r3=000&r6=000&r24=000&vb=0000&vs=0000&mwd=000&pwd=000";
char fonaTime[24];
unsigned long estimatedTime; //This is UNIX Timestamp of the estimated time kept during Power Down modes
unsigned long prevMillis;
unsigned long currentTime;  //the last time read by the fona module
unsigned long sendTime;
unsigned long senseTime;
long duration;
volatile boolean wspulse = false;
volatile boolean wdpulse = false;
volatile unsigned long windSpeedPulse3;
volatile unsigned long windDirectionPulse3;
volatile unsigned long windSpeedPulse2;
volatile unsigned long windSpeedPulse1;
volatile unsigned long windDirectionPulse2;
volatile unsigned long windDirectionPulse1;
volatile unsigned long debounceDir;
volatile unsigned long debounceWin;

unsigned long lastWindSense;
float windSpeedAvg = 0;
float windDirectionAvg = 0;
unsigned int averageCount = 0;
float maxWS = -1;
float minWS = 186;
float maxWD = -1;
float minWD = 361;
char webText[100];
unsigned long timerAdjustment = 0;
unsigned long timerCalibration = 1100;
byte sleepDuration = 8;

short sendInterval[24] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 }; //Interval in Minutes between sending Data
short senseInterval[24] = { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 }; //Interval in Minutes between sensing wind data
short senseDuration[24] = { 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60 }; //duration in Seconds to measure wind data


/*
Each time the windspeed pin goes high it triggers an Interrupt Service Routine (ISR) which stops the current code and stores the time that the pin went high.
The ISRs are functions ISR_for_Speed() and ISR_for_Direction()
This variables used to calculate wind speed and direction are illustrated below.
`
`                            |<----prevWSInterval----->|<------WSInterval------->|
`                            |                         |                         |
`
`  --------------            --------------            WSP-----------            WSC-----------
`  |            |            |            |            |            |            |
`  |            |            |            |            |            |            |
`  |            |            |            |            |            |            |
`---            --------------            --------------            --------------
`
`                                                    ->|-----|<---This duration in relation to WSInterval determines the wind direction.
`
`
`        |<-----prevWDInterval---->|<------WDInterval------->|
`        |                         |                         |
`
`        --------------            WDP-----------            WDC-----------            --------------
`        |            |            |            |            |            |            |
`        |            |            |            |            |            |            |
`        |            |            |            |            |            |            |
`---------            --------------            --------------            --------------

Notes on wind direction calculation:
- WDC is always within the WSInterval.
- The Wind direction is proportional to where WDC sits within the WSInterval.
- In the example above, using text spaces as measurements of time, WDC is 7 units from WSP and WSC is 26 units from WSP. The direction can be calculated as ((WDC-WSP)/(WSC-WSP))*360 or (7/26)*360 = 97 degrees

Pulse Validation Criteria:
1 - Check to make sure that two consecutive wind speed pulse intervals are reasonably similar in duration.
2 - Check to make sure that two consecutive wind direction pulse intervals are reasonably similar in duration.
3 - Check to make sure that the WSInterval represents a duration that is within a usable range.  In this case we use a millisecond duration of 15,000 to 2,000,000.
4 - Check to make sure that the WSInterval is reasonably similar to the WDInterval (this is not yet implemented)
5 - Check to make sure that WDC is greater than WSP and less than WSC (Not implemented explicitly but might be built in already)
If WSP, WSC, WDP, WDC, WSInterval, WDInterval, prevWSInterval, prevWDInteval all meet these criteria, valid wind speed and wind direction can be calculated.

*/

//I plan to rewrite this using two interrupts and 1 or 2 arrays
void ISR_for_Direction1() {
	if (debounceDir < millis()){
		windDirectionPulse1 = micros();
		//not sure if I have to detatch the current interrupt before attaching the next
		detachInterrupt(0);
		attachInterrupt(0, ISR_for_Direction2, RISING);
		debounceDir = millis() + 10;
	}
}

void ISR_for_Direction2() {
	if (debounceDir < millis()){
		windDirectionPulse2 = micros();
		detachInterrupt(0);
		attachInterrupt(0, ISR_for_Direction3, RISING);
		debounceDir = millis() + 10;
	}
}

void ISR_for_Direction3() {
	if (debounceDir < millis()){
		windDirectionPulse3 = millis() * 1000;
		detachInterrupt(0);
		wdpulse = true;
	}
}

void ISR_for_Speed1() {
	if (debounceWin < millis()){
		windSpeedPulse1 = micros();
		detachInterrupt(1);
		attachInterrupt(1, ISR_for_Speed2, RISING);
		debounceWin = millis() + 10;
	}
}

void ISR_for_Speed2() {
	if (debounceWin < millis()){
		windSpeedPulse2 = micros();
		detachInterrupt(1);
		attachInterrupt(1, ISR_for_Speed3, RISING);
		debounceWin = millis() + 10;
	}
}

void ISR_for_Speed3() {
	if (debounceWin < millis()){
		windSpeedPulse3 = micros();
		detachInterrupt(1);
		wspulse = true;
	}
}



void setup()
{
	//define pinmodes
	pinMode(FONA_KEY_PIN, OUTPUT);
	pinMode(FONA_PS_PIN, INPUT);
	pinMode(WindSpeed_PIN, INPUT);
	pinMode(WindDir_PIN, INPUT);
	pinMode(LED, OUTPUT);
	pinMode(Debug_PIN, INPUT);

	Serial.begin(115200);

	//Determine whether to use debugmode or not
	if (digitalRead(Debug_PIN) == HIGH){
		debugMode = true;
		Serial.println(F("--------------------Operating in Debug Mode--------------------"));
	}
	else {
		debugMode = false;
		Serial.println(F("--------------------Operating in Normal Mode--------------------"));
	}


	Serial.println(F("\n-----Starting Boot Sequence-----"));
	Serial.println(F("Checking Battery Voltage"));
	Serial.print(F("  VBat = "));
	Serial.print(getVoltage(V_BAT_PIN));
	Serial.print(F("  VSol = "));
	Serial.print(getVoltage(V_SOLAR_PIN));

	boolean startFONAError = !startFONA(); //Power up the cell module

	//Insert IMEI# into URL's
	for (byte n = 28; n <= 42; n++){
		getSettingsURL[n + 5] = IMEI[n - 28];
		sendLogEntryURL[n] = IMEI[n - 28];
		sendDataURL[n + 1] = IMEI[n - 28];
	}

	boolean networkStartError = !startNetwork();

	//set the fona time and start time estimation
	boolean setTimeError = !setFonaTime(); //make sure the cell can get the time and sets the variable fonaTime
	currentTime = timeToSeconds(fonaTime);
	estimatedTime = currentTime * 1000; //starts estimated time to the current time
	prevMillis = millis(); //this is a setup fir the updateTimeEst() func

	//// Get Wakeup Schedule
	//if (debugMode){
	//	Serial.println(F("Operating in Debug Mode"));
	//	getWakeSchedule();
	//}
	//else {
	//	Serial.println(F("Using Default Wake Schedule"));
	//}

	boolean endNetworkError = !endNetwork();
	boolean stopFonaError = stopFona();

	updateTimeEst(false);
	sendTime = getTargetTime(sendInterval, estimatedTime / 1000); //Returns value in Seconds format
	senseTime = getTargetTime(senseInterval, estimatedTime / 1000);//Returns value in Seconds format
	duration = getSenseDuration(senseDuration, estimatedTime / 1000);//Returns value in Seconds format

	Serial.println(F("\n----------End of Setup----------"));
	Serial.println(F("Checking for errors"));
	if (startFONAError){
		Serial.println(F("  Error starting up Fona"));
	}
	if (networkStartError){
		Serial.println(F("  Error connecting to the network"));
	}
	if (setTimeError){
		Serial.println(F("  Error setting the FONA time"));
	}
	if (endNetworkError){
		Serial.println(F("  Error stopping the network"));
	}
	if (stopFonaError){
		Serial.println(F("  Error turning off the FONA"));
	}
	if (!startFONAError && !networkStartError && !setTimeError && !endNetworkError && !stopFonaError){
		Serial.println(F("  No errors found!"));
	}

	updateTimeEst(false);
	Serial.println(F("Important Times"));
	Serial.print(F("  estimatedTime: ")); secondsToText(estimatedTime / 1000); Serial.println();
	Serial.print(F("  senseTime: ")); secondsToText(senseTime); Serial.println();
	Serial.print(F("  sendTime: ")); secondsToText(sendTime); Serial.println();

}

void loop()
{
	if (getVoltage(V_BAT_PIN) > 3600 && estimatedTime > senseTime * 1000 && estimatedTime <= (senseTime * 1000 + duration * 1000)){
		senseWind();
	}	
	
	//Send Data
	if (estimatedTime > sendTime * 1000 && getVoltage(V_BAT_PIN) > 3600){
		sendData();
	}

	sleep();
}

//gets the voltage of pin using a voltage divider equation
short getVoltage(byte pin){
	/*
	Voltage Divider Diagram
	Vin----
	`     |
	`     R1
	`	  |
	`	  ----Vout
	`	  |
	`     R2
	`     |
	GND-------GND
	*/
	const short coreV = 3300; //This is the operating voltage of the Arduino in mV
	const short R1 = 100; //Kohms
	const short R2 = 270; //Kohms
	const short steps = 1023; // number of steps (range) in the Analog to Digital Conversion
	delay(100); //Time to allow the Analog to Digital Converter to recharge within the Arduino.  This is only needed because we have such high resistor values.
	return (long(analogRead(pin)) * long(coreV) * long(R1 + R2)) / (long(steps) * long(R2));
}

boolean startFONA(){
	Serial.print(F("\n\n-----Attempting to start FONA-----"));
	boolean started = false;
	byte attempts = 1;
	while (!started && attempts < 10){
		Serial.print(F("\n  Attempt #")); Serial.println(attempts);
		delay(100);
		if (digitalRead(FONA_PS_PIN) != HIGH){ //if FONA is off
			digitalWrite(FONA_KEY_PIN, HIGH); //Turn on the FONA
			delay(2000); //Per the FONA manual, it takes a 2 second pulse to turn on the FONA
			digitalWrite(FONA_KEY_PIN, LOW); //Put the pin back to low to save power and so we are prepared to turn off the FONA later with another 2 second pulse
		}
		Serial.println("    Initializing FONA");
		fonaSS.begin(4800); //Start FONA Communication
		//See if the FONA is responding
		if (!fona.begin(fonaSS)){
			Serial.print(F("      Couldn't find FONA"));
			attempts++;
			delay(2000);
		}
		else {
			Serial.println(F("      FONA has started"));
			byte imeiLen = fona.getIMEI(IMEI); //Get the IMEI Number.  We'll use this as a unique identifier for the Weather Station.
			delay(100);
			Serial.print(F("      FONA IMEI = ")); Serial.print(IMEI);
			fona.setGPRSNetworkSettings(F("wap.cingular"), F("wap@cingulargprs.com"), F("cingular1"));
			started = true;
		}
		if (attempts == 10){
			Serial.println(F("\n----------FONA FAILED TO START----------\n"));
		}
	}
	return started;
}

//connects fona to the network
boolean startNetwork(){
	byte netstat; //variable to hold network status
	Serial.print(F("\n\n-----Checking network status-----\n"));
	byte attempts = 1;
	while (netstat != 1 && netstat != 5 && attempts < 10){
		Serial.print(F("  Attempt #")); Serial.print(attempts);
		netstat = fona.getNetworkStatus(); //get the network status	
		Serial.print(F("\n    Network status ")); Serial.print(netstat); Serial.print(F(": "));
		switch (netstat) {
		case 0:
			Serial.println(F("  Not registered"));
			break;
		case 1:
			Serial.println(F("  Registered (home)"));
			break;
		case 2:
			Serial.println(F("  Not registered (searching)"));
			break;
		case 3:
			Serial.println(F("  Denied"));
			break;
		case 4:
			Serial.println(F("  Unknown"));
			break;
		case 5:
			Serial.println(F("  Registered roaming"));
			break;
		}
		if (netstat != 5 && netstat != 1){
			delay(4000);
			attempts++;
		}
		if (attempts == 10){
			Serial.println(F("----------Failed to Connect to the network----------"));
		}
	}

	delay(1000);

	//Turn on GPRS
	attempts = 1;
	boolean started = false;
	Serial.print(F("\n-----Turning on GPRS-----"));
	while (!started && attempts < 10 && (netstat == 1 || netstat == 5)){
		Serial.print(F("\nAttempt #")); Serial.print(attempts);
		started = fona.enableGPRS(true);
		delay(2000); //give time for the fona to do the command
		if (!started){ //turn on the GPRS Radio
			Serial.print(F("\n  GPRS failed to turn on"));
			attempts++;
			delay(2000);
			started = fona.enableGPRS(false);
			started = false;
			delay(2000);
		}
		else{
			Serial.print(F("\n  GPRS successfully turned on"));
		}
	}

	return started;
}

boolean setFonaTime(){
	Serial.println(F("\n\n-----Setting FONA Time-----"));
	boolean fonaNTPSync = fona.enableNTPTimeSync(true, F("pool.ntp.org"));
	delay(2000); //Allow time for the command to complete
	if (!fonaNTPSync){
		Serial.println(F("  NTP Time Sync Failed"));
	}
	else {
		Serial.println(F("  NTP Time Sync Successful"));
	}
	fona.getTime(fonaTime, 23);  // make sure replybuffer is at least 23 bytes!
	Serial.print(F("    Time = ")); Serial.println(fonaTime);

	return fonaNTPSync;
}

boolean getFonaTime(){
	long LastTime = timeToSeconds(fonaTime);//Grab the previous time first for comparison
	fona.getTime(fonaTime, 23);  // make sure replybuffer is at least 23 bytes!
	if (LastTime == timeToSeconds(fonaTime)){//If the new time is the same as the current time then we didn't get a good reading from FONA.
		Serial.println(F("  Failed to retrieve FONA Time - Trying Again"));
		fona.getTime(fonaTime, 23);  // Try once more.
		if (LastTime == timeToSeconds(fonaTime)){
			Serial.println(F("  Unable to retrieve FONA Time"));
			return false; //we will keep rolling with the estimated time for another cycle.
		}
	}
	else {
		//Serial.print(F("  Retrieved FONA Time = ")); Serial.println(fonaTime);
		return true;
	}
}

unsigned long timeToSeconds(char timeStamp[]){
	//Serial.println(F("\n----Converting Time to Seconds-----"));
	//Serial.print(F("Converting: ")); Serial.print(timeStamp);
	char temp[3];
	//changes the hour
	temp[0] = timeStamp[10];
	temp[1] = timeStamp[11];
	long hour = atoi(temp);
	//Serial.print(F("\n  Hour = ")); Serial.print(hour);
	//changes the minute
	temp[0] = timeStamp[13];
	temp[1] = timeStamp[14];
	long min = atoi(temp);
	//Serial.print(F("\n  Minute = ")); Serial.print(min);
	//changes the second
	temp[0] = timeStamp[16];
	temp[1] = timeStamp[17];
	long sec = atoi(temp);
	//Serial.print(F("\n  Second = ")); Serial.print(sec);

	return (sec + (min + hour * 60) * 60);
}

void secondsToText(long Seconds){
	byte Hour = float(Seconds) / 3600.0;
	byte Min = (Seconds - float(Hour)*3600.0) / 60.0;
	byte Sec = (Seconds - float(Hour)*3600.0 - float(Min)*60.0);
	Serial.print(Hour);
	Serial.print(F(":"));
	if (Min < 10){
		Serial.print(F("0"));
	}
	Serial.print(Min);
	Serial.print(F(":"));
	if (Sec < 10){
		Serial.print(F("0"));
	}
	Serial.print(Sec);
}

//turn of GPRS
boolean endNetwork(){
	byte attempts = 1;
	boolean stopped = false;
	Serial.println(F("\n-----Turning off GPRS-----"));
	while (!stopped && attempts < 10) {
		Serial.print(F("Attempt #")); Serial.print(attempts);
		stopped = fona.enableGPRS(false);
		delay(2000);
		if (!stopped){
			Serial.print(F("\n  GPRS failed to turn off"));
			attempts++;
			delay(2000);
		}
		else{
			Serial.print(F("\n  GPRS successfully turned off"));
		}
	}

	return stopped;
}


boolean stopFona(){
	Serial.println(F("T  urning off Fona"));
	byte attempts = 1;
	boolean running = HIGH;
	while (running && attempts < 10) {
		Serial.print(F("    Attempt #")); Serial.print(attempts);
		running = digitalRead(FONA_PS_PIN);
		if (running != LOW){ //check to see if it's already off
			digitalWrite(FONA_KEY_PIN, HIGH); //FONA starts when the KEY pin is pulled low but we use a transistor to reverse this so it start when pulled high
			delay(2000); //Per the FONA manual, it takes a 2 second pulse to turn on the FONA.
			digitalWrite(FONA_KEY_PIN, LOW); //Put the pin back to low to save power and so we are prepared to turn off the FONA later with another 2 second pulse.
			delay(1000); //Pause for a bit to give FONA time to turn off
		}
		else{
			Serial.print(F("\n      FONA is already off"));
		}
		running = digitalRead(FONA_PS_PIN);
		if (running == HIGH){
			Serial.print(F("\n      Failed to turn off FONA"));
			delay(2000);
			attempts++;
		}
		else{
			Serial.println(F("\n      Successfully Turned off FONA"));
		}
		delay(100); //allow the serial port to chatch up
	}

	return running;
}

void updateTimeEst(boolean sentData){
	//Serial.println(F("\n\n-----Updating estimated time-----"));
	estimatedTime += millis() - prevMillis;
	prevMillis = millis();
	if (sentData == true && estimatedTime >= 86400000){
		estimatedTime = 0;
	}
}

long getTargetTime(short schedule[24], long currentTime){
	//Serial.print(F("  Current time is: ")); Serial.println(currentTime);
	long targetTime = 0;
	for (int hour = 0; hour < 24; hour++){
		for (int min = 0; min <= 60; min += schedule[hour]){
			if (currentTime > targetTime){
				targetTime += schedule[hour] * 60;
			}
		}
	}
	//Serial.print(F("  Target time is: ")); Serial.println(targetTime);
	//Serial.print(F("  Minutes until target time: ")); Serial.println((targetTime - currentTime) / 60);
	return targetTime;
}

short getSenseDuration(short Schedule[24], long currentTime){
	byte hour = float(currentTime) / 3600.0;
	return Schedule[hour];
}

//www.agrolan.co.il/UploadProductFiles/AWVPRO.pdf
double intervalToMPH(unsigned long WS_Pulse_Interval){
	double RPS = (double)1000000.0 / (double)WS_Pulse_Interval;

	if (RPS >= 0.1 & RPS < 3.23){
		return -0.1095 * (RPS * RPS) + (2.9318 * RPS) - 0.1412;
	}
	else if (RPS >= 3.23 & RPS < 54.362){
		return 0.0052 * (RPS * RPS) + (2.1980 * RPS) + 1.1091;
	}
	else if (RPS >= 54.362 & RPS >= 66.332){
		return 0.1104 * (RPS * RPS) - (9.5685 * RPS) + 329.87;
	}
	else if (RPS > 66.332){
		return 185;
	}
	else {
		return 0;
	}
}

int intervalToDir(long WS_Pulse_Interval, long DIR_Pulse_Interval){
	return 360.0 * ((double)DIR_Pulse_Interval / (double)WS_Pulse_Interval);
}

float averager(unsigned int averageCount, float currAverage, float NewValue){
	return (NewValue * (1 / float(averageCount))) + (currAverage * ((float(averageCount) - 1.0) / float(averageCount)));
}

boolean getDHTData(DHT22 ThisDHT22, short &DHTtemp, short &DHThumidity){
	byte NumOfTries = 3;//How many times should we try to get data?
	DHT22_ERROR_t errorCode;
	for (byte i = 1; i <= NumOfTries; i++){
		Serial.print(F("Requesting DHT22 data. Attempt #")); Serial.println(i);
		errorCode = ThisDHT22.readData();
		switch (errorCode)
		{
		case DHT_ERROR_NONE:
			Serial.print(F("Got Data "));
			DHTtemp = (ThisDHT22.getTemperatureC() * 9.0 / 5.0 + 32.0) * 10;
			Serial.print(DHTtemp);
			Serial.print(F("F/10 "));
			DHThumidity = ThisDHT22.getHumidity() * 10;
			Serial.print(DHThumidity);
			Serial.println(F("%/10"));
			return true;
			break;
		case DHT_ERROR_CHECKSUM:
			Serial.print(F("check sum error "));
			Serial.print(ThisDHT22.getTemperatureC());
			Serial.print(F("C "));
			Serial.print(ThisDHT22.getHumidity());
			Serial.println(F("%"));
			break;
		case DHT_BUS_HUNG:
			Serial.println(F("BUS Hung "));
			break;
		case DHT_ERROR_NOT_PRESENT:
			Serial.println(F("Not Present "));
			break;
		case DHT_ERROR_ACK_TOO_LONG:
			Serial.println(F("ACK time out "));
			break;
		case DHT_ERROR_SYNC_TIMEOUT:
			Serial.println(F("Sync Timeout "));
			break;
		case DHT_ERROR_DATA_TIMEOUT:
			Serial.println(F("Data Timeout "));
			break;
		case DHT_ERROR_TOOQUICK:
			Serial.println(F("Polled to quick "));
			break;
		}
		delay(2000); //Wait before we try again
	}
	Serial.println(F("Unable to retrieve data from DHT22"));
	return false;
}

char* intToText(short Integer, short NumOfDigits, char intToText[5]){
	short Digit;
	for (int i = 1; i <= NumOfDigits; i++){
		Digit = Integer / pow(10, NumOfDigits - i);
		Integer = Integer - Digit * pow(10, NumOfDigits - i);
		intToText[i - 1] = byte(Digit) + 48;
	}
	return intToText;
}

void buildDataURL(char Data[20], byte beginPos, byte endPos){
	for (int i = beginPos; i <= endPos; i++){
		sendDataURL[i] = Data[i - beginPos];
	}
}

//ask about this
boolean readURL(char url[160]){  // read website URL
	int16_t PageLength = 0;//Number of Characters the website returned

	uint16_t statuscode;
	for (byte i = 0; i < 100; i++){
		webText[i] = ' ';//specifically this
	}
	flushSerial();
	Serial.println(F("Connecting to Website"));
	Serial.print(F("http://")); Serial.println(url);
	if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&PageLength)) {
		return false;
	}
	if (statuscode != 200){
		return false;
	}
	int i = 0;
	while (i < PageLength) {
		while (fona.available()) {
			char c = fona.read();
			webText[i] = c;
			i++;
		}
	}
	fona.HTTP_GET_end();
	return true;
}

void flushSerial() {
	while (Serial.available())
		Serial.read();
}

void sleep(){
	//This routine puts the Arduino to sleep for 8 seconds and maintains the estimatedTime
	if (estimatedTime < senseTime * 1000){//This condition skips the sleep cycle if it's really time to sense.
		Serial.println(F("\n-----Sleep Mode-----"));
		Serial.print(F("  Estimated time is: ")); secondsToText(estimatedTime / 1000); Serial.println();
		Serial.println(F("    Powering down for ~8sec..."));
		delay(10); //Allow serial commands to complete.	
		estimatedTime += millis() - prevMillis; //Do one last estimate of the time before going to sleep
		// The millis timer (driven by 8mhz external crystal) halts while the CPU is powered off.
		LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);//Code stops here until 8s timer expires. Time keeping during power down is based upon inaccurate clock internal to PLC instead of external 8mhz crystal.
		prevMillis = millis();//millis timer is working again so grab the current millis value
		estimatedTime += sleepDuration * timerCalibration; //add the calibrated sleep duration back into the estimated time.
		Serial.println(F("  Awake"));
		Serial.print(F("    timerCalibration: ")); Serial.println(timerCalibration);
		Serial.print(F("    Current time is ")); secondsToText(estimatedTime / 1000); Serial.println();
	}
}

void print_WS_and_WD_Data(){
	Serial.print(F("  Current Time = ")); secondsToText(estimatedTime / 1000); Serial.println();
	Serial.print(F("    WD1="));
	Serial.print(windDirectionPulse1);
	Serial.print(F(" WD2="));
	Serial.print(windDirectionPulse2);
	Serial.print(F(" WD3="));
	Serial.println(windDirectionPulse3);
	Serial.print(F("    WS1="));
	Serial.print(windSpeedPulse1);
	Serial.print(F(" WS2="));
	Serial.print(windSpeedPulse2);
	Serial.print(F(" WS3="));
	Serial.println(windSpeedPulse3);
}

void senseWind(){
	//sense wind loop
	boolean firstWindLoop = true;
	boolean ranWindLoop = false;
	//Serial.println(F("\n----------Start of Loop----------\n"));
	while (estimatedTime > senseTime * 1000 && estimatedTime <= (senseTime * 1000 + duration * 1000)){
		if (firstWindLoop){
			Serial.print(F("Sensing duration is: "));  Serial.println(duration);
			firstWindLoop = false;
			ranWindLoop = true;
		}

		Serial.println(F("\n-----Sensing Wind-----"));
		wspulse = false;
		wdpulse = false;

		attachInterrupt(0, ISR_for_Direction1, RISING);
		attachInterrupt(1, ISR_for_Speed1, RISING);

		//waits for the pulses
		while (estimatedTime <= (senseTime * 1000 + duration * 1000) && !wspulse || !wdpulse){
			updateTimeEst(false);
		}

		long prevWDInterval = windDirectionPulse2 - windDirectionPulse1;;
		long prevWSInterval = windSpeedPulse2 - windSpeedPulse1;
		long WDInterval = windDirectionPulse3 - windDirectionPulse2;
		long WSInterval = windSpeedPulse3 - windSpeedPulse2;
		long dirSearch;
		boolean canCalculate = false;

		if (windDirectionPulse3>windSpeedPulse2 && windDirectionPulse3 < windSpeedPulse3){
			dirSearch = windDirectionPulse3 - windSpeedPulse2;
			canCalculate = true;
		}
		else if (windDirectionPulse2>windSpeedPulse2 && windDirectionPulse2 < windSpeedPulse3){
			dirSearch = windDirectionPulse2 - windSpeedPulse2;
			canCalculate = true;
		}
		else{
			Serial.println(F("Other Situation"));
		}
		print_WS_and_WD_Data();
		Serial.println(F("  calculating Wind speed and direction"));
		if ((WSInterval / abs(WSInterval - prevWSInterval) > 4) && (WDInterval / abs(WDInterval - prevWDInterval) > 4) && WSInterval > 15000 && WSInterval < 2000000 && canCalculate){ //Checking to see if pulses are within spec.
			Serial.println(F("    Valid Pulse"));
			averageCount++;
			float windSpeed = intervalToMPH(WSInterval);
			float windDir = intervalToDir(WSInterval, dirSearch);
			windSpeedAvg = averager(averageCount, windSpeedAvg, windSpeed);
			windDirectionAvg = averager(averageCount, windDirectionAvg, windDir);
			Serial.print(F("      windSpeed: ")); Serial.println(windSpeed);
			Serial.print(F("      windDir: ")); Serial.println(windDir);
			if (windSpeed > maxWS){
				maxWS = windSpeed;
				Serial.print(F("      maxWS: ")); Serial.println(maxWS);
			}
			if (windSpeed < minWS){
				minWS = windSpeed;
				Serial.print(F("      minWS: ")); Serial.println(minWS);
			}

			if (windDir > maxWD){
				maxWD = windDir;
				Serial.print(F("      maxWD: ")); Serial.println(maxWD);
			}
			if (windDir < minWD){
				minWD = windDir;
				Serial.print(F("      minWD: ")); Serial.println(minWD);
			}
		}
		else{
			Serial.println(F("  Invalid Pulse"));
		}
		lastWindSense = estimatedTime;

		updateTimeEst(false);

	}//End of sense wind loop

	if (ranWindLoop){
		detachInterrupt(0); //Detach Interrupt for Wind Direction Sensor
		detachInterrupt(1); //Detach Interrupt for Wind Speed Sensor	
		senseTime = getTargetTime(senseInterval, estimatedTime / 1000);//Returns value in Seconds format
		duration = getSenseDuration(senseDuration, estimatedTime / 1000);//Returns value in Seconds format
		Serial.println(F("  Done Sensing Wind"));
		Serial.print(F("  Next senseTime: ")); secondsToText(senseTime); Serial.println();
		Serial.print(F("  Next sendTime: ")); secondsToText(sendTime); Serial.println();
	}
}

void sendData(){
	
		Serial.println(F("\n-----Send Data and calibrate Time-----"));
		Serial.println(F("  Sending Data"));

		//byte URL_slp[2] = { 113, 116 };		//pressure
		//byte URL_r3[2] = { 121, 123 };		//3 hour rain
		//byte URL_r6[2] = { 128, 130 };		//six hour rain
		//byte URL_r24[2] = { 136, 138 };		//24 hour rain

		short temp1 = 0;
		short temp2 = 0;
		short humidity1;
		short humidity2 = 0;
		char textOut[5];

		//get Humidity data		
		if (!getDHTData(dht1, temp1, humidity1)){
			//send error for getting temp and humidity
		}

		//start putting stuff into urls
		buildDataURL(intToText(minWS * 10, 4, textOut), 49, 52);	//mws
		buildDataURL(intToText(windSpeedAvg * 10, 4, textOut), 58, 61);		//aws
		buildDataURL(intToText(maxWS * 10, 4, textOut), 67, 70);		//pws
		buildDataURL(intToText(windDirectionAvg, 3, textOut), 75, 77);		//wd
		buildDataURL(intToText(minWD, 3, textOut), 160, 162);		//mwd
		buildDataURL(intToText(maxWD, 3, textOut), 168, 170);		//pwd
		buildDataURL(intToText(temp1, 3, textOut), 82, 85);		//t1
		buildDataURL(intToText(temp2, 3, textOut), 90, 93);		//t2
		buildDataURL(intToText(humidity1, 3, textOut), 98, 100);	//h1
		buildDataURL(intToText(humidity2, 3, textOut), 105, 107);	//h2
		buildDataURL(intToText(getVoltage(V_BAT_PIN), 4, textOut), 143, 146);	//vb
		buildDataURL(intToText(getVoltage(V_SOLAR_PIN), 4, textOut), 151, 154);	//vs

		Serial.println(sendDataURL);

		startFONA();
		startNetwork();

		//send data code
		readURL(sendDataURL);

		Serial.println(F("\n-----Updating time estimate-----"));
		if (getFonaTime()){ //Get actual time from the FONA realtime clock.
			stopFona();
			unsigned long previousTime = currentTime;
			currentTime = timeToSeconds(fonaTime); //Convert seconds to millis			
			Serial.print(F("  currentTime: ")); Serial.println(currentTime);
			Serial.print(F("    =")); secondsToText(currentTime); Serial.println();
			Serial.print(F("  estimatedTime: ")); Serial.println(estimatedTime / 1000);
			Serial.print(F("    =")); secondsToText(estimatedTime / 1000); Serial.println();
			Serial.print(F("  previousTime: ")); Serial.println(previousTime);
			unsigned long tempTime = estimatedTime;
			updateTimeEst(false);
			long timeError;

			if (currentTime >= estimatedTime){
				timeError = -(currentTime - (estimatedTime / 1000));
			}
			else{
				timeError = ((estimatedTime / 1000) - currentTime);
			}

			Serial.print(F("Time Error: ")); Serial.println(timeError);
			/*timerAdjustment = currentTime/(estimatedTime/1000);
			Serial.print(F("  timerAdjustment: ")); Serial.println(timerAdjustment);*/

			if (timerAdjustment >= 40){
				timerAdjustment = 40;
			}
			else if (timerAdjustment <= -40){
				timerAdjustment = -40;
			}
			timerCalibration += timeError;
			if (timerCalibration >= 1300){
				timerCalibration = 1300;
			}
			else if (timerCalibration <= 700){
				timerCalibration = 700;
			}

			/*Serial.print(F("Timer Adjustment = ")); Serial.println(timerAdjustment);*/
			Serial.print(F("Timer Calibration = ")); Serial.println(timerCalibration);
		}
		else {
			stopFona();
		}
		averageCount = 0;
		windSpeedAvg = 0;
		windDirectionAvg = 0;
		minWS = 186;
		maxWS = -1;
		maxWD = -1;
		minWD = 361;;
		updateTimeEst(true);
		sendTime = getTargetTime(sendInterval, estimatedTime / 1000);//Returns value in Seconds format
		senseTime = getTargetTime(senseInterval, estimatedTime / 1000);//Returns value in Seconds format
		duration = getSenseDuration(senseDuration, estimatedTime / 1000);//Returns value in Seconds format
		Serial.print(F("  Next senseTime: ")); secondsToText(senseTime); Serial.println();
		Serial.print(F("  Next sendTime: ")); secondsToText(sendTime); Serial.println();
}