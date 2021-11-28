#ifndef PhotoTransistor_h
#define PhotoTransistor_h

#include <Arduino.h>
#include "Config.h"



class PhotoTransistor
{
  public:
    void setup(uint8_t pin);
    // void identify_signal();
    int read();
    void test();

  private:
    uint8_t _pin;
};


void PhotoTransistor::setup(uint8_t pin) {
    pinMode(pin, INPUT);
    _pin = pin;
}


int PhotoTransistor::read() {
    return analogRead(_pin);
}


void PhotoTransistor::test() {
    for (int i = 0; i < 500; i++) {
        Serial.println(read());
        delay(100);
    }   
    Serial.println("PhotoTransistor fine.");
}


#endif
