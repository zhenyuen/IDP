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
    void down(int angle);
    void search();
    void test();
  
  private:
    uint8_t _pin; 
    uint8_t _pos;
};


void ClawRaiser::setup(uint8_t pin) {
    _pin = pin;
    _pos = CLAW_RAISER_UP;
    servo.attach(_pin);
    servo.write(_pos);
}


void ClawRaiser::up() {
  for (_pos = _pos; _pos <= CLAW_RAISER_UP; _pos += 5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(_pos); // tell servo to go to position in variable '_pos'
    delay(CLAW_RAISER_DELAY);                       // waits 15ms for the servo to reach the position
  }
}


void ClawRaiser::search() {
  servo.write(CLAW_RAISER_SEARCH);
}


void ClawRaiser::down(int angle=CLAW_RAISER_DOWN) {
  for (_pos = _pos; _pos >= angle; _pos -= 5) { // goes from 180 degrees to 0 degrees
    servo.write(_pos);              // tell servo to go to position in variable '_pos'
    delay(100);                       // waits 15ms for the servo to reach the position
  }
}


void ClawRaiser::test() {
    up();
    delay(1000);
    down();
    Serial.println("Servo fine.");
}

#endif
