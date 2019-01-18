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
#include <RTCZero.h>
#include "DFRobotDFPlayerMini.h"
/* Create an rtc object */
RTCZero rtc;

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 50;
const byte hours = 18;

/* Change these values to set the current initial date */
const byte day = 11;
const byte month = 02;
const byte year = 18;

long randNumber;
//SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

bool matched = false;

unsigned char pinNumber;

const int sensorPin = 1;
int lightLevel, high = 0, low = 1023;

void setup()
{
  Serial.begin(9600);
  //SerialUSB.begin(115200);
//while(!SerialUSB);
//  
//  SerialUSB.println();
//  SerialUSB.println(F("DFRobot DFPlayer Mini Demo"));
//  SerialUSB.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  if (!myDFPlayer.begin(Serial)) {  //Use softwareSerial to communicate with mp3.
//    SerialUSB.println(F("Unable to begin:"));
//    SerialUSB.println(F("1.Please recheck the connection!"));
//    SerialUSB.println(F("2.Please insert the SD card!"));
    while(true);
  }
  //SerialUSB.println(F("DFPlayer Mini online."));
//randNumber = random(1, 4);
//SerialUSB.println(randNumber);

  //lightLevel = analogRead(sensorPin);
  //SerialUSB.println(lightLevel);
  rtc.begin();

  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);

  rtc.setAlarmTime(18, 50, 10);
  rtc.enableAlarm(rtc.MATCH_MMSS);

  rtc.attachInterrupt(alarmMatch);

  
  
  myDFPlayer.volume(10);  //Set volume value. From 0 to 30
  //myDFPlayer.play(1);

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
   delay(5000);
   USBDevice.detach();
   rtc.standbyMode();
}

void loop()
{

  
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
  //lightLevel = analogRead(sensorPin);
  //SerialUSB.println(lightLevel);
  if (matched) {
    matched = false;
    lightLevel = analogRead(sensorPin);
    
    if(lightLevel >= 250){ 
      myDFPlayer.volume(30);
      //SerialUSB.println(F("play sound"));
      int alarmMinutes = rtc.getMinutes();
      alarmMinutes += 7;
      if (alarmMinutes >= 60) {
        alarmMinutes -= 60;
      }
      //SerialUSB.println(F("play sound"));
     // randNumber = random(1, 4);
      myDFPlayer.play(1);
      rtc.setAlarmTime(rtc.getHours(), alarmMinutes, rtc.getSeconds());
      rtc.standbyMode();    // Sleep until next alarm match
    }else{
      int alarmMinutes = rtc.getMinutes();
      alarmMinutes += 30;
      if (alarmMinutes >= 60) {
        alarmMinutes -= 60;
      }
      //SerialUSB.println(F("night time"));
      rtc.setAlarmTime(rtc.getHours(), alarmMinutes, rtc.getSeconds());
      rtc.standbyMode();    // Sleep until next alarm match
    }
  }
}


void alarmMatch()
{
  matched = true;
  
}

void printDetail(uint8_t type, int value){
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
