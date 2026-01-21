#include <EEPROM.h>
#include <Wire.h>
#include "Adafruit_SHTC3.h"
#include <ArtronShop_SHT45.h>
#include <PID_v1.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <arduino-timer.h>
// #include "temperatureRegulator"
#define Nextion Serial5
#define TIME_HEADER  "T"   // Header tag for serial time sync message

float tempTargetValue = 25, humidityTargetValue = 40;
// AlarmId morning;
// AlarmId evening;

double T_heatingPad = 30.0,  outputHeatingPad = 0.0, setpointHeatingPad = 600;
double T_Air = 20, humidityAir = 50, outputAir = 0, outputHumidity = 0, setpointAir = 24, setpointHumidity = 60;
float T_insert = 0, humidityInsert= 0;
double pT = 2, iT = 0.5, dT = 0.001;
double pT_Air = 10, iT_Air = 0.5, dT_Air = 0.005;
double pT_Humidity = 10, iT_Humidity = 0.5, dT_Humidity = 0.005;
bool heating = 1;
bool humidifying = 1;
bool whiteOn = 1;
bool redOn = 1;
bool insertOn = 1;
int maxBrightnessWhite = 102, maxBrightnessRed = 102, maxBrightnessInsert = 255;
int heaterDuty = 0, humidityDuty = 0;
PID heatingMatTempPID(&T_heatingPad, &outputHeatingPad, &setpointHeatingPad, pT, iT, dT, DIRECT);
PID airTempPID(&T_Air, &outputAir, &setpointAir, pT_Air, iT_Air, dT_Air, DIRECT);
PID airHumidityPID(&humidityAir, &outputHumidity, &setpointHumidity, pT_Humidity, iT_Humidity, dT_Humidity, DIRECT);
float humidity_targetValue = 40.0, humidity_actualValue = 0.0;
auto airSensorTimer = timer_create_default();
auto writeValuesToSerialTimer = timer_create_default();

int fan0 = 29;
int fanInsert = 28;
int pwmHumidity = 0;//11;

int dimOutWhite = 3;
int dimOutRed = 4;
int dimOutInsert = 1;
int dimPercentRed = 0, dimPercentWhite = 0, dimPercentInsert = 0;
int dimValueWhite = 2, dimValueRed = 2, dimValueInsert = 2;
int count = 0;
int counterHalfWaves = 0, counterHalfWavesHumidity = 0;
int zeroDetectPin = 22;
int heatingPadTempPin = 23;
float heatingPadTemp = 900;
bool zeroState = 0;
TwoWire *SHTC3_2;
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
ArtronShop_SHT45 sht45(&Wire2, 0x44); // SHT45-AD1B => 0x44
int counterDisplayUpdate = 0;

float floatDummy = 25.1;
boolean dummy  = false;
unsigned long timeAlt = 0;

void Nextion_Display_serial_listen() {
  if (Nextion.available() > 0) {
    //Serial.println(Nextion.readString());
    String txtValue = Nextion.readStringUntil(' ');
    String txtIdent = Nextion.readStringUntil(';');
    Serial.print(txtValue.toInt());
    Serial.print(" ... ");
    Serial.println(txtIdent);
        //write setpoints to EEPROM
        //0: temperature integer part & on/off on MSB
        //1: temperature 2 post decimal values (as int)
        //2: humidity integer part & on/off on MSB
        //3: humidity 2 post decimal values (as int)
        //4: dimming white (0 - 100)
        //5: dimming red (0 - 100)
    if(txtIdent=="%"){
      setpointHumidity = txtValue.toFloat();
      if (setpointHumidity < 0) setpointHumidity = 0;
      if (setpointHumidity >100) setpointHumidity = 100;
      Serial.print("Luftfeuchtigkeit  ");
      Serial.print(setpointHumidity);
      Serial.println(txtIdent);
      digitalClockDisplay();
      EEPROM.write(2, (humidifying << 7) ^ int(setpointHumidity));
      EEPROM.write(3,int((setpointHumidity - int(setpointHumidity))*100));
    }
    else if(txtIdent=="W"){
      dimPercentWhite = txtValue.toInt();
      dimValueWhite = map(dimPercentWhite, 0,100,0, maxBrightnessWhite);//10 limits brightness
      // analogWrite(dimOutWhite, dimValueWhite);
      setLedBrightness();
      Serial.print("LED W  ");
      Serial.print(dimPercentWhite);
      Serial.print(" ");
      Serial.println(dimValueWhite);
      EEPROM.write(4,dimPercentWhite ^ (whiteOn << 7));
      Serial.println(dimPercentWhite ^ (whiteOn << 7), BIN);
    }
    else if(txtIdent=="R"){
      dimPercentRed = txtValue.toInt();
      dimValueRed = map(dimPercentRed, 0,100,0, maxBrightnessRed);
      setLedBrightness();
      Serial.print("LED R  ");
      Serial.println(dimValueRed);
      EEPROM.write(5,dimPercentRed ^ (redOn << 7));
    }
    else if(txtIdent=="I"){
      dimPercentInsert = txtValue.toInt();
      dimValueInsert = map(dimPercentInsert, 0,100,0, maxBrightnessInsert);
      setLedBrightness();
      Serial.print("LED I  ");
      Serial.println(dimValueInsert);
      EEPROM.write(6,dimPercentInsert ^ (insertOn << 7));
    }
    else if(txtIdent=="Wt"){
      whiteOn = !whiteOn;
      setLedBrightness();
      // analogWrite(dimOutWhite, 255 - (255 - dimValueWhite) * whiteOn);
      // Serial.print("LED W toggled");
      sendLedStateToNextion();
      EEPROM.write(4,dimPercentWhite ^ (whiteOn << 7));
      // Serial.println(dimPercentWhite ^ (whiteOn << 7), BIN);
    }
    else if(txtIdent=="Rt"){
      redOn = !redOn;
      setLedBrightness();
      // analogWrite(dimOutRed, 255 - (255 - dimValueRed) * redOn);
      sendLedStateToNextion();
      EEPROM.write(5,dimPercentRed ^ (redOn << 7));
    }
    else if(txtIdent=="It"){
      insertOn = !insertOn;
      setLedBrightness();
      digitalWrite(fanInsert, insertOn);
      // analogWrite(dimOutRed, 255 - (255 - dimValueRed) * redOn);
      sendLedStateToNextion();
      EEPROM.write(6,dimPercentInsert ^ (insertOn << 7));
    }
    else if(txtIdent=="h"){
      heating = txtValue.toInt();
      EEPROM.write(0,(heating << 7) ^ int(setpointAir));
      Serial.print("Heater ");
      Serial.println(txtValue);
      heatingMatTempPID.SetMode(heating); //set mode according to heating bool - 0 >> manual mode, 1 >> auto mode
      airTempPID.SetMode(heating);
      setpointHeatingPad = 600; //setpoint low so false heating is less probable
      heaterDuty = 0;
    }    
    else if(txtIdent=="hu"){
      humidifying = txtValue.toInt();
      EEPROM.write(2, (humidifying << 7) ^ int(setpointHumidity));
      Serial.print("Heater ");
      Serial.println(txtValue);
      airHumidityPID.SetMode(humidifying);
      humidityDuty = 0;
    }       
    else{
      if(txtIdent.endsWith("C")){
        setpointAir = txtValue.toFloat();
        if (setpointAir < 20) setpointAir = 20;
        if (setpointAir > 50) setpointAir = 50;
        Serial.print("Temperature  ");
        Serial.print(txtValue);
        Serial.println(" °C"); 
        EEPROM.write(0,(heating << 7) ^ int(setpointAir));
        EEPROM.write(1,int((setpointAir - int(setpointAir))*100));
        // Serial.print(int(setpointAir));
      }
    }
  }
}
bool writeValuesToSerial(void*){
  Serial.print("Set_Air: ");
  Serial.print(setpointAir);
  Serial.print(", T_Air: ");
  Serial.print(T_Air);
  Serial.print(", Set_Hum: ");
  Serial.print(setpointHumidity);
  Serial.print(", humidity: ");
  Serial.print(humidityAir);
  Serial.print(", T_Ins: ");
  Serial.print(T_insert);
  Serial.print(", Hum_Ins: ");
  Serial.print(humidityInsert);
  Serial.print(", Bright_R: ");
  Serial.print(dimPercentRed * redOn);
  Serial.print(", Bright_W: ");
  Serial.print(dimPercentWhite * whiteOn);
  Serial.print(", Bright_I: ");
  Serial.println(dimPercentInsert * insertOn);
  return true;
}
bool getAirData(void *){
  sensors_event_t humidity, temp;
  shtc3.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
  if(sht45.measure()){
    T_insert = sht45.temperature();
    humidityInsert = sht45.humidity();
  }
  T_Air = temp.temperature;
  humidityAir = humidity.relative_humidity;
  writeTemperatureToNextion(float(T_Air));
  writeHumidityToNextion(float(humidityAir));
  writeTemperatureInsertToNextion(float(T_insert));
  writeHumidityInsertToNextion(float(humidityInsert));
  return true;
}
void airRegulator(){
  if(airTempPID.Compute()){
    //Serial.println(humidityAir);
    setpointHeatingPad = map(outputAir, 0, 255, 550, 1000);
  }
  if(heatingMatTempPID.Compute()){
    T_heatingPad= analogRead(heatingPadTempPin);
    heatingPadTemp =  (heatingPadTemp * 99 + analogRead(heatingPadTempPin)) / 100;
    heaterDuty = map( outputHeatingPad, 0, 255, 0, 100);
    counterHalfWaves = 0;
  }
  if (airHumidityPID.Compute()){
    // Serial.print(" humidifying: ");
    // Serial.println(humidifying);
    // Serial.print(" humidityDuty: ");
    // Serial.println(humidityDuty);
    counterHalfWavesHumidity = 0;
    humidityDuty = map(outputHumidity, 0, 255, 0, 1000);
  }
}
void printRegulators(){
  Serial.print("T_Air: ");
  Serial.print(T_Air);
  Serial.print(", setpointAir: ");
  Serial.print(setpointAir);
  Serial.print(", outputAir: ");
  Serial.print(outputAir);
  Serial.print(", T_heatingPad:");
  Serial.print(T_heatingPad);
  Serial.print(",  outputHeatingPad:");
  Serial.print( outputHeatingPad);
  Serial.print(", setpointHeatingPad:");
  Serial.print(setpointHeatingPad);
  Serial.print(", humidity:");
  Serial.print(humidityAir);
  Serial.print(",  outputHumidity");
  Serial.print( outputHumidity);
  Serial.print(", setpointHumidity:");
  Serial.println(setpointHumidity);
}

void writeTemperatureToNextion(float  temperature) {
  String str = "mainPage.tActualTemp.txt=\"" ;
  str += temperature;
  str +=" \xBA C\""; // ° - Zeichen character Encoding iso-8859-1
  commandToNextion(str);
}
void writeHumidityToNextion(float  humidity) {
  String str =  "mainPage.tActualHumi.txt=\"" ;
  str += humidity;
  str += " %\"";
  commandToNextion(str);
}
void writeTemperatureInsertToNextion(float  temperature) {
  String str = "mainPage.T_Insert.txt=\"" ;
  str += temperature;
  str +=" \xBA C\""; // ° - Zeichen character Encoding iso-8859-1
  commandToNextion(str);
}
void writeHumidityInsertToNextion(float  humidity) {
  String str =  "mainPage.humidityInsert.txt=\"" ;
  str += humidity;
  str += " %\"";
  commandToNextion(str);
}
void writeTemperatureSetpointToNextion() {
  String str = "mainPage.tTargetTemp.txt=\"" ;
  str += setpointAir;
  str +=" \xBA C\""; // ° - Zeichen character Encoding iso-8859-1
  commandToNextion(str);
}
void writeHumiditySetpointToNextion() {
  String str =  "mainPage.tTargetHumi.txt=\"" ;
  str += setpointHumidity;
  str += " %\"";
  commandToNextion(str);
}
void writeAirSwitchesToNextion(){
  String str =  "mainPage.switchHeater.val=" ;
  str += heating;
  commandToNextion(str);
  str =  "mainPage.swHumidifier.val=" ;
  str += humidifying;
  commandToNextion(str);
}
void writeWhiteBrightnessToNextion() {
  String str =  "mainPage.dimWhite.val=" ;
  str += dimPercentWhite;
  commandToNextion(str);
}
void writeRedBrightnessToNextion() {
  String str =  "mainPage.dimRed.val=" ;
  str += dimPercentRed;
  commandToNextion(str);
}
void writeInsertBrightnessToNextion() {
  String str =  "mainPage.dimInsert.val=" ;
  str += dimPercentInsert;
  commandToNextion(str);
}
void whiteStateToNextion(){
  String str =  "mainPage.toggleWhite.val=" ;
  str += whiteOn;
  commandToNextion(str);
}
void redStateToNextion(){
  String str =  "mainPage.toggleRed.val=" ;
  str += redOn;
  commandToNextion(str);
}
void insertStateToNextion(){
  String str =  "mainPage.toggleInsert.val=" ;
  str += insertOn;
  commandToNextion(str);
}
void sendLedStateToNextion(){
  String str =  "mainPage.toggleWhite.val=" ;
  str += whiteOn;
  commandToNextion(str);
  str =  "mainPage.toggleRed.val=" ;
  str += redOn;
  commandToNextion(str);
  str =  "mainPage.toggleInsert.val=" ;
  str += insertOn;
  commandToNextion(str);
}
void commandToNextion(String str){
  Nextion.print(str);
  Nextion.print("\xFF\xFF\xFF");
}

void countHalfWaves(){
  counterHalfWaves +=1;
  counterHalfWavesHumidity +=1;

  if (counterHalfWaves < heaterDuty){
    digitalWrite(5, HIGH);
  }
  else{
    digitalWrite(5, LOW);
  }
  if (counterHalfWavesHumidity < humidityDuty){
    analogWrite(pwmHumidity, 128);
  }
  else{
    analogWrite(pwmHumidity, 0);
  }
}
void setLedBrightness(){
  analogWrite(dimOutRed, dimValueRed * redOn); // invert and set value depending on ON/OFF state
  analogWrite(dimOutWhite, dimValueWhite * whiteOn);
  analogWrite(dimOutInsert, dimValueInsert * insertOn);
}

void setup() {
  pinMode(fan0, OUTPUT);
  pinMode(fanInsert, OUTPUT);
  pinMode(pwmHumidity, OUTPUT);
  pinMode(dimOutRed, OUTPUT);
  pinMode(dimOutWhite, OUTPUT);
  pinMode(dimOutInsert, OUTPUT);
  analogWrite(dimOutWhite, 0); // set to "off" (inverted PWM, 255) before reading the stored values later
  analogWrite(dimOutRed, 0);
  analogWrite(dimOutInsert, 0);
  
  Serial.begin(9600);
  Nextion.begin(115200);
  
  pinMode(zeroDetectPin, INPUT);
  analogWriteFrequency(pwmHumidity, 104000);
  analogWriteResolution(8);
  
  analogWriteFrequency(dimOutWhite, 100);//min frequency defined by led driver datasheet, used due to low speed op-amp
  analogWriteFrequency(dimOutRed, 100);
  analogWriteFrequency(dimOutInsert, 100);
  analogWrite(fan0, 175);

  //analogWrite(fan0, 220);
  // analogWrite(pwmHumidity, 128);
//  Serial.begin(115200);

 
  airSensorTimer.every(1000, getAirData);
  writeValuesToSerialTimer.every(10000, writeValuesToSerial);

  Serial.println("SHTC3 test");
  if (!shtc3.begin()) Serial.println("Couldn't find SHTC3");
  else Serial.println("Found SHTC3 sensor");
  Wire2.begin();
  if (!sht45.begin()) {
    Serial.println("SHT45 not found !");
    delay(1000);
  }
  attachInterrupt(22, countHalfWaves, CHANGE);

  //Read setpoints from EEPROM
  //0: temperature integer part & heating bool as MSB
  //1: temperature 2 post decimal values (as int)
  //2: humidity integer part & humidifying bool as MSB
  //3: humidity 2 post decimal values (as int)
  //4: dimming white (0 - 100) and on/off as MSB
  //5: dimming red (0 - 100) and on/off as MSB
  setpointAir = float(EEPROM.read(0) & ~(1 << 7))  + float(~(1<<7) & EEPROM.read(1))/100;
  setpointHumidity = float(EEPROM.read(2) & ~(1 << 7)) + float(EEPROM.read(3))/100;
  heating = EEPROM.read(0) & (1 << 7);
  humidifying = EEPROM.read(2) & (1 << 7);
  dimPercentWhite = ~(1<<7) & EEPROM.read(4);
  dimPercentRed = ~(1<<7) & EEPROM.read(5);
  dimPercentInsert = ~(1<<7) & EEPROM.read(6);
  whiteOn = (1<<7) & EEPROM.read(4);
  redOn = (1<<7) & EEPROM.read(5);
  insertOn = (1<<7) & EEPROM.read(6);
  dimValueWhite = map(dimPercentWhite, 0,100,0, maxBrightnessWhite);
  dimValueRed = map(dimPercentRed, 0,100,0, maxBrightnessRed);
  dimValueInsert = map(dimPercentInsert, 0,100,0, maxBrightnessInsert);
  setLedBrightness();
  digitalWrite(fanInsert, insertOn);

  heatingMatTempPID.SetMode(heating);
  heatingMatTempPID.SetSampleTime(1000);
  airTempPID.SetMode(heating);
  airTempPID.SetSampleTime(1000);
  airHumidityPID.SetMode(humidifying);
  airHumidityPID.SetSampleTime(10000);

  // analogWrite(dimOutRed, 255 - (255 - dimValueRed) * redOn); // invert and switch to value depending on 
  // analogWrite(dimOutWhite, 255 - (255 - dimValueWhite) * whiteOn);
  delay(1500);
  //write EEPROM stored values to display
  writeAirSwitchesToNextion();
  writeTemperatureSetpointToNextion();
  writeHumiditySetpointToNextion();
  writeWhiteBrightnessToNextion();
  writeRedBrightnessToNextion();
  writeInsertBrightnessToNextion();
  whiteStateToNextion();
  redStateToNextion();
  insertStateToNextion();
  setSyncProvider(getTeensy3Time);
  Alarm.alarmRepeat(8,0,0, boxLightsOn);
  Alarm.alarmRepeat(18,0,0, boxLightsOff);
  // setSyncProvider(getTeensy3Time);
  digitalClockDisplay();
  Serial.println("finished setup...");
}
void loop() {
    if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
  Nextion_Display_serial_listen();
  airSensorTimer.tick();
  writeValuesToSerialTimer.tick();

  airRegulator();
  //  digitalClockDisplay();
   Alarm.delay(0); // wait one second between clock display

}

void boxLightsOff(){
  Serial.println("lights off");
  redOn = 0;
  whiteOn = 0;
  setLedBrightness();
  whiteStateToNextion();
  redStateToNextion();
}
void boxLightsOn(){
  Serial.println("lights on");
  redOn = 1;
  whiteOn = 1;
  setLedBrightness();
  whiteStateToNextion();
  redStateToNextion();
}
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}
void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}
