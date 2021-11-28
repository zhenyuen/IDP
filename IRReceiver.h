#ifndef IRReceiver_h
#define IRReceiver_h

#include <Arduino.h>
#include "Config.h"


class IRReceiver
{
  public:
    void setup(uint8_t pin);
    bool read();
    void test();

  private:
    uint8_t _pin;
};


void IRReceiver::setup(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
    _pin = pin;
}


bool IRReceiver::read() {
    return !digitalRead(_pin);
}


void IRReceiver::test() {
    for (int i = 0; i < 200; i++) {
        Serial.println(read());
        delay(100);
    }
    Serial.println("IR receiver fine.");
}


#endif
