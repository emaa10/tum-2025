#include "Arduino.h"

void setup() {
    Serial.begin(9600);
}

void loop() {
    delay(50);
    Serial.println("working");
}