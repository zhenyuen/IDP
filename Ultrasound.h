#ifndef Ultrasound_h
#define Ultrasound_h


#include <Arduino.h>
#include "Config.h"


class Ultrasound
{
    public:
        void setup(uint8_t echo_pin, uint8_t trig_pin);
        float read();
        void test();

    private:
        uint8_t _echo_pin;
        uint8_t _trig_pin;
};


void Ultrasound::setup(uint8_t echo_pin, uint8_t trig_pin) {
    pinMode(trig_pin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echo_pin, INPUT); // Sets the echoPin as an INPUT
    _echo_pin = echo_pin;
    _trig_pin = trig_pin;
}


float Ultrasound::read() {
    // Clears the trigPin condition
    digitalWrite(_trig_pin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig_pin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(_echo_pin, HIGH);
    // Calculating the distance
    float distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    return distance;
}


void Ultrasound::test() {
    for (int i = 0; i < 50; i++) {
        Serial.println(read());
        delay(500);
    }
    Serial.println("Ultrasound fine.");
}


#endif
