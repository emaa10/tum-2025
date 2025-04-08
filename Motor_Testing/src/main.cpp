/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <DynamixelShield.h>
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

VL53L0X sensor1;

#define XSHUT_FRONT 1
#define XSHUT_BACK 2
//#define LED_BUILTIN 13

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

//Motor Bezeichnungen
const uint8_t Motor_links = 2; 
const uint8_t Motor_rechts = 1;

const float DXL_PROTOCOL_VERSION = 1.0; //Wichtig, nicht 2.0

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  DEBUG_SERIAL.begin(19200); //Die Baudrate nehmen sonst stirb er iwi
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.begin();
  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor1.startContinuous();
  digitalWrite(LED_BUILTIN, LOW);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(Motor_links);
  dxl.ping(Motor_rechts);

  // Turn off torque when configuring items in EEPROM area, wichtig is OP_VELOCITY wenn wir UNIT_PERCENT nutzen
  dxl.torqueOff(Motor_links);
  dxl.torqueOff(Motor_rechts);
  dxl.setOperatingMode(Motor_links, OP_VELOCITY);
  dxl.setOperatingMode(Motor_rechts, OP_VELOCITY);
  dxl.torqueOn(Motor_links);
  dxl.torqueOn(Motor_rechts);
}

//Speeds, leebmanns rentner benz
int Motorlinks_Geschw = -50; 
int Motorrechts_Geschw = 50;  

void drivegay() {
  dxl.setGoalVelocity(Motor_links, Motorlinks_Geschw, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, Motorrechts_Geschw, UNIT_PERCENT);
}

void driveslow() {
  dxl.setGoalVelocity(Motor_links, Motorlinks_Geschw/2, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, Motorrechts_Geschw/2, UNIT_PERCENT);
}

void turnleft() {
  dxl.setGoalVelocity(Motor_links, 20, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, 99, UNIT_PERCENT);
}

void turnright() {
  dxl.setGoalVelocity(Motor_links, -0, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, -80, UNIT_PERCENT);
}

void turnsharpleft() {
  dxl.setGoalVelocity(Motor_links, 0, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, Motorrechts_Geschw, UNIT_PERCENT);
}

void dance() {
  dxl.setGoalVelocity(Motor_links, -0, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, -0, UNIT_PERCENT);
}

void driveBergauf() {
  dxl.setGoalVelocity(Motor_links, -0, UNIT_PERCENT); //voller drehmoment
  dxl.setGoalVelocity(Motor_rechts, 90, UNIT_PERCENT); //langsame dreh zahl damit er hochkommt
}  

unsigned long getDistance() {
  unsigned long distance = sensor1.readRangeContinuousMillimeters();
  if (sensor1.timeoutOccurred()) {
    return 500; //des kannst keinem erzählen
  } else {
    return distance;
  }
}

int turn_staerke = 1.2;
int verzörgern = 1;
int turns = 0;

void loop() {
 if (getDistance() > 200){
    if (false){
      turnleft();
      delay(3000);
      drivegay();
      delay(2000);
    }
    turnleft();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(turn_staerke * 100);
    drivegay();
    delay(100 * verzörgern);
  }

  if (getDistance() < 100) {
    turnright();
    digitalWrite(LED_BUILTIN, LOW);
    delay(turn_staerke * 100);
    drivegay();
    delay(100 * verzörgern);
  }
  else{
    drivegay();
  }
  delay(10);
}