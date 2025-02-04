#include <DynamixelShield.h>
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

VL53L0X sensor1;
VL53L0X sensor2;

enum Motor {
  Motor_Links,
  Motor_Rechts,
  check,
};

#define XSHUT_FRONT 1
#define XSHUT_BACK 2

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
  //Sensors Start:
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_BACK, OUTPUT);
  digitalWrite(XSHUT_BACK, LOW);
  digitalWrite(XSHUT_BACK, HIGH);
  delay(10);
  sensor2.setAddress(0x30);
  sensor2.startContinuous();
}
float stop = -100;
float msl = -15.0;  //max speed links
float msr = 85.0;   //max speed rechts

int Motorlinks_Geschw = 100;   //Variable für lesen aktuelle RPM Motor links
int Motorrechts_Geschw = 100;  //Variable für lesen aktuelle RPM Motor rechts


void motorController(double Speed, Motor motorlocation) {
  double correctedSpeed = 0;
  switch (motorlocation)
  {
  case Motor_Links:
    if (Speed > 0){
      correctedSpeed = 100 - Speed; //greislig ich weiß
    }
    else {
      correctedSpeed = (100 - Speed) * -1; //greislig ich weiß
    }
    dxl.setGoalVelocity(Motor_links, correctedSpeed, UNIT_PERCENT);
    break;
  case Motor_Rechts:
    if (Speed < 0){
      correctedSpeed = 100 - Speed; //greislig ich weiß
    }
    else {
      correctedSpeed = (100 - Speed) * -1; //greislig ich weiß
    }
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
  for (int i = 0; i < 100; i++)
  {
    motorController(i, Motor_Links);
    motorController(i, Motor_Rechts);
    delay(100);
  }
  for (int i = 0; i < 50; i++)
  {
    motorController(NULL, check);
    delay(10);
  }

  while(true){yield();}
    
  delay(550);
}