/***************************************************
  DFPlayer - A Mini MP3 Player For Arduino
  <https://www.dfrobot.com/index.php?route=product/product&product_id=1121>

 ***************************************************
  This example shows the basic function of library for DFPlayer.

  Created 2016-12-07
  By [Angelo qiao](Angelo.qiao@dfrobot.com)

  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
  1.Connection and Diagram can be found here
  <https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
  2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

#include "Arduino.h"
//#include "SoftwareSerial.h"

#include <SerialFlash.h>
#include <RTCZero.h>
#include "DFRobotDFPlayerMini.h"

/* Create an rtc object */
RTCZero rtc;

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 57;
const byte hours = 0;

/* Change these values to set the current initial date */
const byte day = 01;
const byte month = 01;
const byte year = 18;

/* DFPlayer Mini setup */
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);
bool playSound = true;

unsigned char pinNumber;

#define LIGHTSENSORPIN A1
int lightLevel;
const int lightLevelThreshold = 255;
const int awakeIntervalMin = 3; //measured in minutes
const int awakeIntervalSec = 30; //measured in minutes
const int sleepInterval = 30; //measured in minutes
int alarmIntervalMin = awakeIntervalMin;
int alarmIntervalSec = awakeIntervalSec;
const int maxRunTime = 8; // measured in hours

bool debugging = false;
bool tempDebugging = false;
/* thermistor setup */
// which analog pin to connect
#define THERMISTORPIN A2
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 50
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3892
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

uint16_t samples[NUMSAMPLES];

/* Heater setup */
int heaterPin = 5;
float heaterStartTemp = -24;
bool heaterOn = false;
int heaterValue = 0;

/* Sound file setup */
int numSoundFiles = 2; 
int currentFileNumber = 1;

void setup()
{
  //Serial communication is always required for communicating with the DFPlayer mini
  Serial.begin(9600);
  /* Thermistor specific setup */
  analogReference(AR_DEFAULT);

  if (debugging) {
    //SerialUSB is for debugging using the rocketscream boards
    SerialUSB.begin(115200);
    while (!SerialUSB);

    SerialUSB.println();
    SerialUSB.println(F("DFRobot DFPlayer Mini Demo"));
    SerialUSB.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  }
  //Connect to the DFplayer mini
  if (!myDFPlayer.begin(Serial)) {  //Use softwareSerial to communicate with mp3.
    if (debugging) {
      SerialUSB.println(F("Unable to begin:"));
      SerialUSB.println(F("1.Please recheck the connection!"));
      SerialUSB.println(F("2.Please insert the SD card!"));
    }
    while (true);
  }
  if (debugging) {
    SerialUSB.println(F("DFPlayer Mini online."));
    lightLevel = analogRead(LIGHTSENSORPIN);
    SerialUSB.println(lightLevel);
   int temp = myDFPlayer.readFileCounts();
   SerialUSB.print(F("DFPlayer Mini file count: "));
   SerialUSB.println(temp);
   numSoundFiles = temp;
  }

  



  myDFPlayer.volume(15);  //Set volume value. From 0 to 30
  for (int i = 1; i < numSoundFiles+1; i++)
  {
    //delay(2500);
    myDFPlayer.play(i);
    delay(10000);
    
  }
  rtc.begin();

  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);

  setSoundAlarm();
  rtc.enableAlarm(rtc.MATCH_MMSS);

  rtc.attachInterrupt(wakeupAlarm);

  //Low power settings
  for (pinNumber = 0; pinNumber < 23; pinNumber++)
  {
    pinMode(pinNumber, INPUT_PULLUP);
  }

  for (pinNumber = 32; pinNumber < 42; pinNumber++)
  {
    pinMode(pinNumber, INPUT_PULLUP);
  }

  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  analogWrite(heaterPin, 0);
  delay(15000);
  if (!debugging) {
    USBDevice.detach();
  }
}

void loop()
{
  if (heaterOn) {
    analogWrite(heaterPin, heaterValue);
  }

  if (playSound) {
    myDFPlayer.volume(30);
    if (debugging) {
    myDFPlayer.volume(5);
    }
    
    myDFPlayer.play(currentFileNumber);
    if(currentFileNumber==numSoundFiles){
      currentFileNumber = 1;
    }else{
      currentFileNumber++;
    }
    playSound = false;
    setSoundAlarm();

    if (!heaterOn) {
      if (debugging) {
        SerialUSB.println("standby");
      }
      rtc.standbyMode();    // Sleep until next alarm match
    }
  }
}


void wakeupAlarm()
{

  if (debugging) {
    SerialUSB.println("wakeupAlarm");
  }
  /* Check ambient */


  uint8_t i;
  float average;
  pinMode(THERMISTORPIN, INPUT);
  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    if (tempDebugging) {
      samples[i] = 975;
    } else {
      samples[i] = analogRead(THERMISTORPIN);
    }
  }
  pinMode(THERMISTORPIN, INPUT_PULLUP);
  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  if (steinhart < heaterStartTemp) {
    heaterOn = true;
    if (steinhart > -35) {
      heaterValue = 60 + (int)90 * (((steinhart * -1) + heaterStartTemp) / 11 );
    } else {
      heaterValue = 150;
    }

    //pinMode(heaterPin, OUTPUT);
  } else {
    heaterOn = false;
    heaterValue = 0;
    //pinMode(heaterPin, INPUT_PULLUP);
    analogWrite(heaterPin, 0);
  };


  if (debugging) {
    SerialUSB.print("Temperature ");
    SerialUSB.print(steinhart);
    SerialUSB.println(" *C");
    SerialUSB.print("Heater Value: ");
    SerialUSB.println(heaterValue);
  }
  //TODO:: Check if first playback and store current time



  /* Check light levels */
  lightLevel = analogRead(LIGHTSENSORPIN);
  if (lightLevel >= lightLevelThreshold) {
    playSound = true;
  } else {
    playSound = false;
  }

  //TODO::Check total playback time
  if (playSound || heaterOn) {
    alarmIntervalMin = awakeIntervalMin;
  } else {
    alarmIntervalMin = sleepInterval;
  }
  if (tempDebugging) {
    SerialUSB.print("alarmInterval ");
    SerialUSB.println(alarmIntervalMin);
  }


}

void setSoundAlarm(){
  int alarmHours = rtc.getHours();

    
    int alarmMinutes = rtc.getMinutes();
    int alarmSeconds = rtc.getSeconds();
    if (debugging) {
    SerialUSB.print("alarm mins: ");
    SerialUSB.println(alarmMinutes);
    
    SerialUSB.print("alarm secs: ");
    SerialUSB.println(alarmSeconds);
    }
    alarmSeconds += alarmIntervalSec;
    if (alarmSeconds >= 60) {
      alarmSeconds -= 60;
      alarmMinutes++;
    }
    
    alarmMinutes += alarmIntervalMin;
    if (alarmMinutes >= 60) {
      alarmMinutes -= 60;
    }

     
   if (debugging) {
    SerialUSB.print("alarm mins: ");
    SerialUSB.println(alarmMinutes);
    
    SerialUSB.print("alarm secs: ");
    SerialUSB.println(alarmSeconds);
    }
    rtc.setAlarmMinutes(alarmMinutes);
    rtc.setAlarmSeconds(alarmSeconds); 
}

void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      SerialUSB.println(F("Time Out!"));
      break;
    case WrongStack:
      SerialUSB.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      SerialUSB.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      SerialUSB.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      SerialUSB.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      SerialUSB.print(F("Number:"));
      SerialUSB.print(value);
      SerialUSB.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      SerialUSB.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          SerialUSB.println(F("Card not found"));
          break;
        case Sleeping:
          SerialUSB.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          SerialUSB.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          SerialUSB.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          SerialUSB.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          SerialUSB.println(F("Cannot Find File"));
          break;
        case Advertise:
          SerialUSB.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
