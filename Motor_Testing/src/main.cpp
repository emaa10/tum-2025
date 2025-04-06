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
  dxl.setGoalVelocity(Motor_links, -30, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, 30, UNIT_PERCENT);
}

void turnleft() {
  dxl.setGoalVelocity(Motor_links, -0, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, 99, UNIT_PERCENT);
}

void turnright() {
  dxl.setGoalVelocity(Motor_links, -99, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, -0, UNIT_PERCENT);
}

void turnsharpleft() {
  dxl.setGoalVelocity(Motor_links, 0, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, Motorrechts_Geschw, UNIT_PERCENT);
}

unsigned long getDistance() {
  unsigned long distance = sensor1.readRangeContinuousMillimeters();
  if (sensor1.timeoutOccurred()) {
    return 120; //wenn sensor scheiße macht juckt keinen
  } else {
    return distance;
  }
}

void driveBergauf() {
  dxl.setGoalVelocity(Motor_links, -0, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, 80, UNIT_PERCENT);
}

void loop() {
  unsigned long distance = getDistance();
 /* if (distance > 200){
    turnleft();
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (distance < 100) {
    turnright();
    digitalWrite(LED_BUILTIN, LOW);
  }
  if (distance > 100 && distance < 200)
  {*/
    driveBergauf();
  //}
  delay(10);
}