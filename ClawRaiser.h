#ifndef ClawRaiser_h
#define ClawRaiser_h

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
 
 
class ClawRaiser {
  public:
    Servo servo;
    void setup(uint8_t pin);
    void up();
    void down();
    void test();
  
  private:
    uint8_t _pin; 
    uint8_t _pos;
};


void ClawRaiser::setup(uint8_t pin) {
    _pin = pin;
    _pos = 60;
    servo.attach(_pin);
    servo.write(_pos);
}


void ClawRaiser::up() {
  for (_pos = 60; _pos <= 140; _pos += 5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(_pos); // tell servo to go to position in variable '_pos'
    delay(CLAW_RAISER_DELAY);                       // waits 15ms for the servo to reach the position
  }
}


void ClawRaiser::down() {
  for (_pos = 140; _pos >= 60; _pos -= 5) { // goes from 180 degrees to 0 degrees
    servo.write(_pos);              // tell servo to go to position in variable '_pos'
    delay(CLAW_RAISER_DELAY);                       // waits 15ms for the servo to reach the position
  }
}


void ClawRaiser::test() {
    up();
    delay(1000);
    down();
    Serial.println("Servo fine.");
}

#endif
