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

enum Motor {
  Motor_Links,
  Motor_Rechts,
  check,
};

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t Motor_links = 2;
const uint8_t Motor_rechts = 1;

const float DXL_PROTOCOL_VERSION = 1.0; //Wichtig, nicht 2.0

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(19200); //Die Baudrate nehmen sonst stirb er iwi

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(Motor_links);
  dxl.ping(Motor_rechts);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(Motor_links);
  dxl.torqueOff(Motor_rechts);
  dxl.setOperatingMode(Motor_links, OP_VELOCITY);
  dxl.setOperatingMode(Motor_rechts, OP_VELOCITY);
  dxl.torqueOn(Motor_links);
  dxl.torqueOn(Motor_rechts);
}
float stop = -100;
float msl = -15.0;  //max speed links
float msr = 85.0;   //max speed rechts

int Motorlinks_Geschw = 100;   //Variable für lesen aktuelle RPM Motor links
int Motorrechts_Geschw = 100;  //Variable für lesen aktuelle RPM Motor rechts


void motorController(double Speed, Motor motorlocation) {
  switch (motorlocation)
  {
  case Motor_Links:
    dxl.setGoalVelocity(Motor_links, Speed, UNIT_PERCENT);
    break;
  case Motor_Rechts:
    double correctedSpeed; 
    correctedSpeed = (100 - Speed) * -1; //greislig ich weiß
    dxl.setGoalVelocity(Motor_rechts, correctedSpeed, UNIT_PERCENT);
    break;
  case check:
    //Control loop
    Motorlinks_Geschw = dxl.getPresentVelocity(1);
    Motorrechts_Geschw = dxl.getPresentVelocity(2);
    dxl.setGoalVelocity(Motor_rechts, Motorrechts_Geschw, UNIT_RPM);
    dxl.setGoalVelocity(Motor_links, Motorlinks_Geschw, UNIT_RPM);
    break;
  }
}

void loop() {
 for (int i = 0; i < 80; i++)
 {
  motorController(i, Motor_Links);
  motorController(i, Motor_Rechts);
  delay(10);
 }
 

  while(true){yield();}
    
  delay(550);
}