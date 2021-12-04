#ifndef LineSensor_h
#define LineSensor_h

#include <Arduino.h>
#include "Config.h"

// DEPRECATED

class LineSensor
{
  public:
    void setup(uint8_t pin);
    bool read();
    void test();

  private:
    uint8_t _pin;
};


void LineSensor::setup(uint8_t pin) {
    pinMode(pin, INPUT);
    _pin = pin; 
}

bool LineSensor::read() {
    return !digitalRead(_pin);
}

void LineSensor::test() {
    while (true) {
        if (read()) {
            Serial.println("White detected");
            break;
        }
        Serial.println("White not detected");
        delay(100);
    };
    while (true) {
        if (!read()) {
            Serial.println("Black detected");
            break;
        }
        Serial.println("Black not detected");
        delay(100);
    };
    Serial.println("Line sensor fine.");
}

#endif
