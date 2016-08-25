// SDL_Arduino_WeatherLink_Tx
// SwitchDoc Labs August 2016
//

#include <JeeLib.h>

#define SOFTWAREVERSION 002

// WIRELESSID is changed if you have more than one unit reporting in the same area.  It is coded in protocol as WIRELESSID*10+SOFTWAREVERSION
#define WIRELESSID 1


#include "Crc16.h"

//Crc 16 library (XModem)
Crc16 crc;

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}
#include <SoftwareSerial.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include "SDL_Arduino_INA3221.h"

SDL_Arduino_INA3221 SunAirPlus;

// the three channels of the INA3221 named for SunAirPlus Solar Power Controller channels (www.switchdoc.com)
#define LIPO_BATTERY_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3

#define ENABLE_RADIO 5

#define WATCHDOG_1 8
#define WATCHDOG_2 9

// Number of milliseconds between data out - set 1000 or or 30000 or 60000 if you are using DS3231
#define SLEEPCYCLE 30000

// Note:  If you are using a External WatchDog Timer, then you should set the timer to exceed SLEEPCYCLE by at least 10%.  If you are using the SwitchDoc Labs Dual WatchDog, SLEEPCYCLE must be less than 240 seconds
//        or what you set the WatchDog timeout interval.

// Note:  If you are using the alarm DS3231 system, you only have the choice of every second or every minute, but you can change this to some degree:


/*
  Values for Alarm 1

  ALM1_EVERY_SECOND -- causes an alarm once per second.
  ALM1_MATCH_SECONDS -- causes an alarm when the seconds match (i.e. once per minute).
  ALM1_MATCH_MINUTES -- causes an alarm when the minutes and seconds match.
  ALM1_MATCH_HOURS -- causes an alarm when the hours and minutes and seconds match.
  ALM1_MATCH_DATE -- causes an alarm when the date of the month and hours and minutes and seconds match.
  ALM1_MATCH_DAY -- causes an alarm when the day of the week and hours and minutes and seconds match.
  Values for Alarm 2

  ALM2_EVERY_MINUTE -- causes an alarm once per minute.
  ALM2_MATCH_MINUTES -- causes an alarm when the minutes match (i.e. once per hour).
  ALM2_MATCH_HOURS -- causes an alarm when the hours and minutes match.
  ALM2_MATCH_DATE -- causes an alarm when the date of the month and hours and minutes match.
  ALM2_MATCH_DAY -- causes an alarm when the day of the week and hours and minutes match.

*/


SoftwareSerial SoftSerial(6, 7); // TX, RX

unsigned long MessageCount = 0;


#define DEBUG



#include "avr/pgmspace.h"

#include "DS3232RTC/DS3232RTC.h"
#include <Time.h>
#include <Wire.h>

#include <ESG_AM2315.h>

typedef enum  {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  RAIN_INTERRUPT,
  ANEMOMETER_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;


// Device Present State Variables

bool SunAirPlus_Present;
bool DS3231_Present;

byte byteBuffer[200]; // contains string to be sent to RX unit

// State Variables
byte Protocol;
long TimeStamp;
int WindDirection;
float AveWindSpeed;
long TotalRainClicks;
float MaxWindGust;
float OutsideTemperature;
float OutsideHumidity;
float BatteryVoltage;
float BatteryCurrent;
float LoadVoltage;
float LoadCurrent;
float SolarPanelVoltage;
float SolarPanelCurrent;
float AuxA;
float AuxB;
int protocolBufferCount;

wakestate wakeState;  // who woke us up?
bool ignore_anemometer_interrupt;

long nextSleepLength;

long lastRainTime;
long windClicks;
long  lastWindTime;
long shortestWindTime;

// Interfaced Devices

#include "WeatherRack.h"

// Grove AM2315 - 5V Arduino Drivers

ESG_AM2315 am2315;

float dataAM2315[2];  //Array to hold data returned by sensor.  [0,1] => [Humidity, Temperature]
boolean OK;  // 1=successful read




//
// Protocol - 1 Byte - 1 - byte
// TimeStamp - 4 Bytes - milliseconds since bootup - long
// WindDirection -2 Byte - instantenous wind direction - int - degrees
// Ave Wind Speed - 4 bytes - average in the sample interval - float - kph
// Rain Total - 4 bytes - clicks since bootup - long - clicks
// Wind Gust - 4 bytes - high during sample interval - float - kph
// Outside Temperature - 4 bytes - instantenous temperature - float - degrees c
// Outside Humidity - 4 bytes - instaneous humidity - float - % RH
// Aux A - 4 Bytes - user defined - float
// Aux B - 4 bytes - user define - float
// Buffer Total Bytes
//
//

int convert4ByteLongVariables(int bufferCount, long myVariable)
{

  int i;

  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;

}

int convert4ByteFloatVariables(int bufferCount, float myVariable)
{
  int i;

  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];


    bufferCount++;
  }

  return bufferCount;
}


int convert2ByteVariables(int bufferCount, int myVariable)
{


  union {
    int a;
    unsigned char bytes[2];
  } thing;

  thing.a = myVariable;


  byteBuffer[bufferCount] = thing.bytes[0];
  bufferCount++;
  byteBuffer[bufferCount] = thing.bytes[1];
  bufferCount++;

  Serial.println("-------");
  Serial.println(thing.bytes[0]);
  Serial.println(thing.bytes[1]);
  Serial.println("------");

  return bufferCount;

}

int convert1ByteVariables(int bufferCount, int myVariable)
{


  byteBuffer[bufferCount] = (byte) myVariable;
  bufferCount++;
  return bufferCount;

}

int checkSum(int bufferCount)
{
  unsigned short checksumValue;
  // calculate checksum
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);
  Serial.print("crc = 0x");
  Serial.println(checksumValue, HEX);

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}

int buildProtocolString()
{

  int bufferCount;


  bufferCount = 0;

  byteBuffer[bufferCount] = 0xAB;
  bufferCount++;
  byteBuffer[bufferCount] = 0x66;
  bufferCount++;
  bufferCount = convert1ByteVariables(bufferCount, Protocol);
  bufferCount = convert4ByteLongVariables(bufferCount, TimeStamp / 100);
  bufferCount = convert2ByteVariables(bufferCount, WindDirection);
  bufferCount = convert4ByteFloatVariables(bufferCount, AveWindSpeed);
  bufferCount = convert4ByteLongVariables(bufferCount, windClicks);
  bufferCount = convert4ByteLongVariables(bufferCount, TotalRainClicks);
  bufferCount = convert4ByteFloatVariables(bufferCount, MaxWindGust);
  bufferCount = convert4ByteFloatVariables(bufferCount, OutsideTemperature);
  bufferCount = convert4ByteFloatVariables(bufferCount, OutsideHumidity);

  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, LoadCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelCurrent);

  bufferCount = convert4ByteFloatVariables(bufferCount, AuxA);
  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);
  protocolBufferCount = bufferCount + 2;
  //     bufferCount = convert1ByteVariables(bufferCount, protocolBufferCount);
  bufferCount = checkSum(bufferCount);





  return bufferCount;


}



void printStringBuffer()
{
  int bufferLength;

  bufferLength = protocolBufferCount;
  int i;
  for (i = 0; i < bufferLength; i++)
  {
    Serial.print("i=");
    Serial.print(i);
    Serial.print(" | ");
    Serial.println(byteBuffer[i], HEX);
  }

}


// DS3231 Library functions



const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

void printDigits(int digits) {
  // utility function for digital clock display: prints an leading 0

  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay() {

  tmElements_t tm;
  RTC.read(tm);

  // digital clock display of the time
  printDigits(tm.Hour);
  Serial.print(":");
  printDigits(tm.Minute);
  Serial.print(":");
  printDigits(tm.Second);
  Serial.print(" ");
  Serial.print(tm.Day);
  Serial.print("/");
  Serial.print(tm.Month);
  Serial.print("/");
  Serial.print(tm.Year + 1970);
  Serial.println();
}

void return2Digits(char returnString[], char *buffer2, int digits)
{
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);


}

void set32KHz(bool setValue)
{

  uint8_t s = RTC.readRTC(RTC_STATUS);

  if (setValue == true)
  {
    s = s | (1 << EN32KHZ);
    RTC.writeRTC(RTC_STATUS, s);

  }
  else
  {
    uint8_t flag;
    flag =  ~(1 << EN32KHZ);
    s = s & flag;
    RTC.writeRTC(RTC_STATUS, s);

  }



}

// AT24C32 EEPROM

#include "AT24C32.h"


void ResetWatchdog()
{
  pinMode(WATCHDOG_1, OUTPUT);
  delay(200);
  pinMode(WATCHDOG_1, INPUT);
  Serial.println("Watchdog1 Reset - Patted the Dog");

}



void setup()
{
  Serial.begin(115200);    // Debugging only

  SoftSerial.begin(9600);

  Serial.println("--------WeatherRack WeatherLink_Tx Started-------");
  // setup initial values of variables

  wakeState = REBOOT;
  nextSleepLength = SLEEPCYCLE;


  Protocol = WIRELESSID * 10 + SOFTWAREVERSION;
  TimeStamp = 0;
  WindDirection = 0;
  AveWindSpeed = 0.0;
  TotalRainClicks = 0;
  MaxWindGust = 0.0;
  OutsideTemperature = 0.0;
  OutsideHumidity = 0.0;
  BatteryVoltage = 0.0;
  BatteryCurrent = 0.0;
  LoadCurrent = 0.0;
  SolarPanelVoltage = 0.0;
  SolarPanelCurrent = 0.0;
  AuxA = 0.0;
  AuxB = 0.0;

  // protocol 1



  // WeatherRack
  lastRainTime = 0;
  lastWindTime = 0;
  shortestWindTime = 10000000;

  pinMode(WATCHDOG_1, INPUT);
  // Just to turn off LED on _1_
  //pinMode(WATCHDOG_2, OUTPUT);
  //digitalWrite(WATCHDOG_2, HIGH);

  Wire.begin();

  // set up interrupts

  pinMode(2, INPUT);     // pinAnem is input to which a switch is connected
  digitalWrite(2, HIGH);   // Configure internal pull-up resistor
  pinMode(3, INPUT);     // pinRain is input to which a switch is connected
  digitalWrite(3, HIGH);   // Configure internal pull-up resistor

  // Now set up Transmitter off
  digitalWrite(ENABLE_RADIO, HIGH);
  pinMode(ENABLE_RADIO, OUTPUT);

  ignore_anemometer_interrupt = false;
  attachInterrupt(0, serviceInterruptAnem, RISING);
  attachInterrupt(1, serviceInterruptRain, FALLING);

  // test for DS3231 Present
  DS3231_Present = false;

  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if (timeStatus() != timeSet)
  {
    Serial.println(F("DS3231 Not Present"));
    DS3231_Present = false;
  }
  else
  {
    Serial.println(F("DS3231 Present"));
    digitalClockDisplay();
    DS3231_Present = true;
    //   Disable SQW

    RTC.squareWave(SQWAVE_NONE);
    set32KHz(false);

    // Now set up the alarm time
    // we only have a few  choices. for repeatable alarms (once per second, once per minute, once per hour, once per day, once per date and once per day of the week).
    // we are just choosing 1 per second or once per minute), you can get clever and use both alarms to do 30 seconds, 30 minutes, etc.
    if (SLEEPCYCLE < 1001)
    {


      // hits every second
      // set the alarm to go off.

      RTC.setAlarm(ALM1_EVERY_SECOND, 0, 0, 0, 0);
      RTC.alarm(ALARM_1);
      RTC.alarm(ALARM_2);
      RTC.alarmInterrupt(ALARM_1, true);


    }
    else if (SLEEPCYCLE < 30001)
    {
      // choose once per 30 seconds

      RTC.setAlarm(ALM1_MATCH_SECONDS, 30, 0, 0, 0);
      RTC.alarm(ALARM_1);
      RTC.setAlarm(ALM2_EVERY_MINUTE , 0, 0, 0, 0);
      RTC.alarm(ALARM_2);
      RTC.alarmInterrupt(ALARM_1, true);
      RTC.alarmInterrupt(ALARM_2, true);

    }
    else // if (SLEEPCYCLE < 60001)
    {
      // choose once per minute

      RTC.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 0);
      RTC.alarm(ALARM_1);
      RTC.alarm(ALARM_2);
      RTC.alarmInterrupt(ALARM_1, true);


    }

    uint8_t readValue;

  }

  uint8_t s = RTC.readRTC(RTC_STATUS);
  Serial.print("RTC_STATUS=");
  Serial.println(s, BIN);
  s = RTC.readRTC(RTC_CONTROL);
  Serial.print("RTC_CONTROL=");
  Serial.println(s, BIN);


  // test for SunAirPlus_Present
  SunAirPlus_Present = false;

  LoadVoltage = SunAirPlus.getBusVoltage_V(OUTPUT_CHANNEL);

  if (LoadVoltage < 0.1)
  {
    SunAirPlus_Present = false;
    Serial.println("SunAirPlus Not Present");
  }
  else
  {
    SunAirPlus_Present = true;
    Serial.println("SunAirPlus Present");
  }




}




void loop()
{

  if (DS3231_Present)
  {
    RTC.get();
    digitalClockDisplay();
  }
  // Only send if source is SLEEP_INTERRUPT

  Serial.print("wakeState=");
  Serial.println(wakeState);



  if ((wakeState == SLEEP_INTERRUPT) || (wakeState == ALARM_INTERRUPT))
  {

    wakeState = NO_INTERRUPT;

    Serial.print(F("MessageCount="));
    Serial.println(MessageCount);

    OK = am2315.readData(dataAM2315);

    if (OK) {
      Serial.print(F("Outside Temperature (C): ")); Serial.println(dataAM2315[1]);
      Serial.print(F("Outside Humidity (%RH): ")); Serial.println(dataAM2315[0]);
      OutsideTemperature = dataAM2315[1];
      OutsideHumidity = dataAM2315[0];

    }
    else
      Serial.println(F("AM2315 not found, check wiring & pullups!"));

    // now read wind vane

    float WindVaneVoltage;
    WindVaneVoltage = analogRead(A1) * (5.0 / 1023.0);
    Serial.print(F("Wind Vane Voltage ="));
    Serial.println(WindVaneVoltage);
    Serial.print(F("Wind Vane Degrees ="));
    WindDirection = voltageToDegrees(WindVaneVoltage, 0.0);
    Serial.println(WindDirection);
    Serial.print(F("TotalRainClicks="));
    Serial.println(TotalRainClicks);
    Serial.print(F("windClicks="));
    Serial.println(windClicks);

    // calculate wind
    AveWindSpeed = (((float)windClicks / 2.0) / (((float)SLEEPCYCLE) / 1000.0)) * 2.4; // 2.4 KPH/click

    Serial.print(F("Average Wind Speed="));
    Serial.print(AveWindSpeed);
    Serial.println(F(" KPH"));

    Serial.print("shortestWindTime (usec)");
    Serial.println(shortestWindTime);

    MaxWindGust = 0.0;

    shortestWindTime = 10000000;

    TimeStamp = millis();

    // if SunAirPlus present, read charge data

    if (SunAirPlus_Present)
    {

      LoadVoltage = SunAirPlus.getBusVoltage_V(OUTPUT_CHANNEL);
      LoadCurrent = SunAirPlus.getCurrent_mA(OUTPUT_CHANNEL);


      BatteryVoltage = SunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
      BatteryCurrent = SunAirPlus.getCurrent_mA(LIPO_BATTERY_CHANNEL);

      SolarPanelVoltage = SunAirPlus.getBusVoltage_V(SOLAR_CELL_CHANNEL);
      SolarPanelCurrent = -SunAirPlus.getCurrent_mA(SOLAR_CELL_CHANNEL);

      Serial.println("");
      Serial.print(F("LIPO_Battery Load Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
      Serial.print(F("LIPO_Battery Current:       ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));
      Serial.println("");

      Serial.print(F("Solar Panel Voltage:   ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
      Serial.print(F("Solar Panel Current:   ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
      Serial.println("");

      Serial.print(F("Load Voltage:   ")); Serial.print(LoadVoltage); Serial.println(F(" V"));
      Serial.print(F("Load Current:   ")); Serial.print(LoadCurrent); Serial.println(F(" mA"));
      Serial.println("");

    }

    // write out the current protocol to message and send.
    int bufferLength;
    bufferLength = buildProtocolString();

    Serial.println(F("----------Sending packet----------"));

    // turn radio on
    digitalWrite(ENABLE_RADIO, LOW);
    delay(100);
    SoftSerial.write((uint8_t *)byteBuffer, bufferLength);

    delay(100);
    digitalWrite(ENABLE_RADIO, HIGH);
    // turn radio off

    MessageCount++;

    windClicks = 0;


  }





  if (DS3231_Present == false)
  {
    if (wakeState != REBOOT)
      wakeState = SLEEP_INTERRUPT;
    long timeBefore;
    long timeAfter;
    timeBefore = millis();
    Serial.print("timeBeforeSleep=");
    Serial.println(timeBefore);
    delay(100);
    // This is what we use for sleep if DS3231 is not present
    if (DS3231_Present == false)
    {
      //Sleepy::loseSomeTime(nextSleepLength);
      for (long i = 0; i < nextSleepLength / 16; ++i)
        Sleepy::loseSomeTime(16);

      wakeState = SLEEP_INTERRUPT;

      Serial.print("Awake now: ");

      timeAfter = millis();
      Serial.print("timeAfterSleep=");
      Serial.println(timeAfter);

      Serial.print("SleepTime = ");
      Serial.println(timeAfter - timeBefore);

      Serial.print("Millis Time: ");
      long time;
      time = millis();
      //prints time since program started
      Serial.println(time / 1000.0);
      Serial.print("2wakeState=");
      Serial.println(wakeState);
    }
  }

  if (DS3231_Present == true)
  {
    // use DS3231 Alarm to Wake up
    Serial.println(F("Using DS3231 to Wake Up"));
    delay(100);
    Sleepy::powerDown ();

    if (RTC.alarm(ALARM_1))
    {
      Serial.println("ALARM_1 Found");
      wakeState = ALARM_INTERRUPT;
      // remove one rain click
      if (TotalRainClicks > 0)
        TotalRainClicks = TotalRainClicks - 1;

    }

    if (RTC.alarm(ALARM_2))
    {
      Serial.println("ALARM_2 Found");
      wakeState = ALARM_INTERRUPT;
      // remove one rain click
      if (TotalRainClicks > 0)
        TotalRainClicks = TotalRainClicks - 1;

    }

  }

  // if it is an anemometer interrupt, do not process anemometer interrupts for 17 msec  debounce

  if (wakeState == ANEMOMETER_INTERRUPT)
  {
    ignore_anemometer_interrupt = true;
    delay(17);
    ignore_anemometer_interrupt = false;
  }

  // Pat the WatchDog
  ResetWatchdog();


}


