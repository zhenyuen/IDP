#ifndef DriveSystem_h
#define DriveSystem_h

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "Config.h"


class DriveSystem
{
  public:
    Adafruit_MotorShield* _afms;
    Adafruit_DCMotor* _dc_m_l;
    Adafruit_DCMotor* _dc_m_r;
    
    void setup(Adafruit_MotorShield* afms, uint8_t dc_m_l_pin, uint8_t dc_m_r_pin);
    void add(Adafruit_MotorShield* afms);
    void forward(uint8_t speed);
    void backward(uint8_t speed);
    void rotate(uint8_t speed, bool cw);
    void steer(uint8_t speed, bool left, float steer_factor, bool forward);
    void brake();  
    void get_speed(uint8_t dc_m_speeds[2]);
    void get_direction(uint8_t dc_m_directions[2]);
    bool get_rotating();
    bool get_steering();
    bool get_forward();
    bool get_backward();
    bool get_brake();
    void test();

  private:
    unsigned long _dc_m_l_rotations;
    unsigned long _dc_m_r_rotations;

    uint8_t _dc_m_r_dir;
    uint8_t _dc_m_r_speed;

    uint8_t _dc_m_l_dir;
    uint8_t _dc_m_l_speed;

    bool _rotating;
    bool _steering;
    bool _forward;
    bool _backward;

};


void DriveSystem::setup(Adafruit_MotorShield* afms, uint8_t dc_m_l_pin, uint8_t dc_m_r_pin) {
    _afms = afms;
    _dc_m_l = afms -> getMotor(dc_m_l_pin);
    _dc_m_r = afms -> getMotor(dc_m_r_pin);
}


void DriveSystem::forward(uint8_t speed) {
    _rotating = false;
    _steering = false;
    _forward = true;
    _backward = false;

    _dc_m_l -> setSpeed(speed);
    _dc_m_r -> setSpeed(speed - 3);
    _dc_m_l -> run(FORWARD);
    _dc_m_r -> run(FORWARD);

    _dc_m_r_dir = FORWARD;
    _dc_m_l_dir = FORWARD;
    _dc_m_l_speed = speed;
    _dc_m_r_speed = speed - 3;
}


void DriveSystem::backward(uint8_t speed) {
    _rotating = false;
    _steering = false;
    _forward = false;
    _backward = true;

    _dc_m_l -> setSpeed(speed - 2);
    _dc_m_r -> setSpeed(speed);
    _dc_m_l -> run(BACKWARD);
    _dc_m_r -> run(BACKWARD);

    _dc_m_r_dir = BACKWARD;
    _dc_m_l_dir = BACKWARD;
    _dc_m_l_speed = speed - 2;
    _dc_m_r_speed = speed;
}


void DriveSystem::get_direction(uint8_t dc_m_directions[2]) {
    dc_m_directions[0] = _dc_m_l_dir;
    dc_m_directions[1] = _dc_m_r_dir;
}


void DriveSystem::get_speed(uint8_t dc_m_speeds[2]) {
    dc_m_speeds[0] = _dc_m_l_speed;
    dc_m_speeds[1] = _dc_m_r_speed;
}


bool DriveSystem::get_steering() {
    return _steering;
}


bool DriveSystem::get_rotating() {
    return _rotating;
}


bool DriveSystem::get_forward() {
    return _forward;
}

bool DriveSystem::get_brake() {
    if (!_rotating && !_steering && !_forward && !_backward) return true;
    return false;
}

bool DriveSystem::get_backward() {
    return _backward;
}


void DriveSystem::rotate(uint8_t speed, bool cw) {
    _rotating = true;
    _steering = false;
    _forward = false;
    _backward = false;

    _dc_m_l -> setSpeed(speed);
    _dc_m_r -> setSpeed(speed);
    
    if (cw) {
        _dc_m_r -> run(BACKWARD);
        _dc_m_l -> run(FORWARD);
        _dc_m_r_dir = BACKWARD;
        _dc_m_l_dir = FORWARD;
    } else {
        _dc_m_r -> run(FORWARD);
        _dc_m_l -> run(BACKWARD);
        _dc_m_r_dir = FORWARD;
        _dc_m_l_dir = BACKWARD;
    }
    _dc_m_l_speed = speed;
    _dc_m_r_speed = speed;
}


void DriveSystem::brake() {
    _rotating = false;
    _steering = false;
    _forward = false;
    _backward = false;

    _dc_m_l -> setSpeed(0);
    _dc_m_r -> setSpeed(0);
    _dc_m_l -> run(RELEASE);
    _dc_m_r -> run(RELEASE);

    _dc_m_l_speed = 0;
    _dc_m_r_speed= 0;
    _dc_m_r_dir = RELEASE;
    _dc_m_l_dir = RELEASE;
    delay(100);
}


void DriveSystem::steer(uint8_t speed, bool left, float steer_factor=STEER_FACTOR, bool forward=true) {
    _rotating = false;
    _steering = true;
    _forward = false;
    _backward = false;

    if (left) {
        _dc_m_l_speed = speed * steer_factor;
        _dc_m_r_speed = speed;
        _dc_m_l -> setSpeed(_dc_m_l_speed);
        _dc_m_r -> setSpeed(_dc_m_r_speed);

    } else {
        _dc_m_r_speed = speed * steer_factor;
        _dc_m_l_speed = speed;
        _dc_m_r -> setSpeed(_dc_m_r_speed);
        _dc_m_l -> setSpeed(_dc_m_l_speed);
    }

    if (forward) {
      _dc_m_r -> run(FORWARD);
      _dc_m_l -> run(FORWARD);
    } else {
      _dc_m_r -> run(BACKWARD);
      _dc_m_l -> run(BACKWARD);
    }
    
    _dc_m_r_dir = FORWARD;
    _dc_m_l_dir = FORWARD;
}


void DriveSystem::test() {
    Serial.println("------- DRIVE SYSTEM TEST -------");
    int speed = 200;
    forward(speed);
    delay(3000);
    backward(speed);
    delay(3000);
    rotate(speed, true);
    delay(3000);
    rotate(speed, false);
    delay(3000);
    brake();
    Serial.println("------- TEST PASSED -------"); 
}

#endif
