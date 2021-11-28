#ifndef Claw_h
#define Claw_h

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
 
 
class Claw {
  public:
    Servo servo;
    void setup(uint8_t pin);
    void open();
    void close();
    void test();
  
  private:
    uint8_t _pin; 
    uint8_t _pos;
};


void Claw::setup(uint8_t pin) {
    _pin = pin;
    _pos = CLAW_OPEN;
    servo.attach(_pin);
    servo.write(_pos);
}


void Claw::close() {
  for (_pos = CLAW_OPEN; _pos <= CLAW_CLOSE; _pos += 5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(_pos); // tell servo to go to position in variable '_pos'
    delay(CLAW_DELAY);                       // waits 15ms for the servo to reach the position
  }
}


void Claw::open() {
  for (_pos = CLAW_CLOSE; _pos >= CLAW_OPEN; _pos -= 5) { // goes from 180 degrees to 0 degrees
    servo.write(_pos);              // tell servo to go to position in variable '_pos'
    delay(CLAW_DELAY);                       // waits 15ms for the servo to reach the position
  }
}


void Claw::test() {
    open();
    Serial.println("Servo open.");
    delay(1000);
    close();
    Serial.println("Servo close.");
    Serial.println("Servo fine.");
}

#endif
