#ifndef Accelerometer_h
#define Accelerometer_h

#include <Arduino.h>
#include "Config.h"
#define SENSITIVITY 420
#define ADC_AMPLITUDE 1024
#define ADC_REF 5000
#define GRAVITY 9.81

class Accelerometer
{
  public:
    void setup(uint8_t pin);
    void test();
    void update_state();
    float get_ay();
    uint8_t get_state();


  private:
    uint8_t _pin;
    uint8_t _state; //1 - Normal, 2 - Ascend, 3 - Descend, 4 - Elevated
    float _y_zero;
    float _acc_hist[ACC_HIST_LEN];
    int _index;
    int _get_index();
};

uint8_t Accelerometer::get_state() {
  return _state;
}

void Accelerometer::setup(uint8_t pin) {
    _pin = pin;
    _index = 0;
    _y_zero = 0;
    _state = NORMAL_STATE;
    
    for (int i=0; i < ACC_HIST_LEN; i++) {
      _acc_hist[i] = 0;
    }
    
    pinMode(_pin, INPUT);
    
    // Compute y_zero
    int y = 0;

    for (int i=0; i < 10; i++) {
      y += analogRead(_pin);
    }
    y /= 10;
    _y_zero = (float) y * ADC_REF / ADC_AMPLITUDE;

    for (int i = 0; i < ACC_HIST_LEN; i++) {
      _acc_hist[i] = _y_zero;
    }
}

void Accelerometer::update_state() {
  int score = 0;
  get_ay();
  //Serial.println("------------------");

  for (int i = 0; i < ACC_HIST_LEN; i++) {
    //Serial.println(_acc_hist[i]);
    if (_state == NORMAL_STATE) {
      if (_acc_hist[i] <= ASCEND_THRESHOLD) {
        score++;
      }
    } else if (_state == ASCEND_STATE || _state == DESCEND_STATE) {
      if (_acc_hist[i] > ASCEND_THRESHOLD && _acc_hist[i] < DESCEND_THRESHOLD) {
        score++;
      }
    } else if (_state == ELEVATED_STATE) {
      if (_acc_hist[i] >= DESCEND_THRESHOLD) {
        score++;
      }
    }
      
  }
  //Serial.println("------------------");

  Serial.println("SCORE IS");
  Serial.println(score);
  int limit = ACC_HIST_LEN / 2;
  if (score > limit) {
    if (_state == NORMAL_STATE) {
      _state = ASCEND_STATE;
    } else if (_state == ASCEND_STATE)  {
      _state = ELEVATED_STATE;
    } else if (_state == DESCEND_STATE) {
      _state = NORMAL_STATE;
    } else if (_state == ELEVATED_STATE) {
      _state = DESCEND_STATE;
    }
  }
}

int Accelerometer::_get_index() {
  int i = _index;
  _index++;
  if (_index == ACC_HIST_LEN) _index = 0;
  return i;
}


float Accelerometer::get_ay() {
  int y = 0;
  
  for (int i=0; i < 10; i++) {
      y += analogRead(_pin);
    }
  y /= 10;
  float y_voltage = (float) y * ADC_REF / ADC_AMPLITUDE;

  float result = (y_voltage - _y_zero) / SENSITIVITY * GRAVITY;
  _acc_hist[_get_index()] = result;
  
  return result;
}


void Accelerometer::test(){
  for (int i = 0; i < 50; i++) {
    Serial.println(get_ay());
    update_state();
    Serial.println(get_state());
    delay(1000);
  }
  Serial.println("Accelerometer fine.");
}


#endif
