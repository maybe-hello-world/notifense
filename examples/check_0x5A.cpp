#include <Wire.h>

void setup() {
    Serial.begin(9600);
    delay(1000);
    Wire.begin();
}

void loop() {
    Wire.beginTransmission(0x5A);
    uint8_t result = Wire.endTransmission();

    Serial.println(result == 0 ? "0x5A OK" : "0x5A MISSING");
    delay(250);
}