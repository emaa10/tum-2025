#include <DynamixelShield.h>

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

void turnleft(int millisec) {
  dxl.setGoalVelocity(Motor_links, 0, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, Motorrechts_Geschw, UNIT_PERCENT);
  delay(10*millisec);
  dxl.setGoalVelocity(Motor_links, Motorlinks_Geschw, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, Motorrechts_Geschw, UNIT_PERCENT);
}

void turnright(int millisec) {
  dxl.setGoalVelocity(Motor_links, Motorlinks_Geschw, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, 0, UNIT_PERCENT);
  delay(10*millisec);
  dxl.setGoalVelocity(Motor_links, Motorlinks_Geschw, UNIT_PERCENT);
  dxl.setGoalVelocity(Motor_rechts, Motorrechts_Geschw, UNIT_PERCENT);
}

void loop() {
    delay(550);
}