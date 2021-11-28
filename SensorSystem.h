#ifndef SensorSystem_h
#define SensorSystem_h


#include <Arduino.h>
#include "LineSensor.h"
#include "Accelerometer.h"
//#include <PhotoInterrupter.h>
#include "Config.h"

class SensorSystem {
  public:
    void setup();
    void add_ls(LineSensor* ls_b, LineSensor* ls_fm, LineSensor* ls_fl, LineSensor* ls_fr);
    void refresh();
    void get_line_sensors(bool readings[4]);
    void test();
    
  private:
    LineSensor* _ls_b;
    LineSensor* _ls_fm;
    LineSensor* _ls_fl;
    LineSensor* _ls_fr;

    bool _ls_hist[LS_HIST_LEN][4];
    int _ls_index;
    int _get_ls_index();


};


void SensorSystem::setup() {
    _ls_index = 0;

}

void SensorSystem::add_ls(LineSensor* ls_b, LineSensor* ls_fm, LineSensor* ls_fl, LineSensor* ls_fr) {
    _ls_b = ls_b;
    _ls_fm = ls_fm;
    _ls_fl = ls_fl;
    _ls_fr = ls_fr;
}
    

void SensorSystem::refresh() {
    int index = _get_ls_index();
    //Serial.println(index);
    _ls_hist[index][0] = _ls_b -> read();
    _ls_hist[index][1] = _ls_fm -> read();
    _ls_hist[index][2] = _ls_fl -> read();
    _ls_hist[index][3] = _ls_fr -> read();
}

int SensorSystem::_get_ls_index() {
    if (_ls_index == LS_HIST_LEN) _ls_index = 0;
    int index = _ls_index;
    _ls_index++;
    return index;
}
    
    
void SensorSystem::get_line_sensors(bool readings[4]) {
    int ls_b_score = 0;
    int ls_fm_score = 0;
    int ls_fl_score = 0;
    int ls_fr_score = 0;
    int threshold = LS_HIST_LEN / 2;

    for (int i = 0; i < LS_HIST_LEN; i++) {
        ls_b_score += _ls_hist[i][0];
        ls_fm_score += _ls_hist[i][1];
        ls_fl_score += _ls_hist[i][2];
        ls_fr_score += _ls_hist[i][3];
    }
    // Serial.println(ls_b_score);
    // Serial.println(ls_fm_score);
    // Serial.println(ls_fl_score);
    // Serial.println(ls_fr_score);

    readings[0] = true? ls_b_score >= threshold : false;
    readings[1] = true? ls_fm_score >= threshold : false;
    readings[2] = true? ls_fl_score >= threshold : false;
    readings[3] = true? ls_fr_score >= threshold : false;
}


void SensorSystem::test() {
    for (int i = 0; i < 5; i++) {
        // Test line sensor
        Serial.println("------- LINE SENSOR TEST -------");
        for (int j = 0; j < LS_HIST_LEN; j++) {
            refresh();
            delay(100);
        }
        bool temp[4];
        get_line_sensors(temp);
        Serial.println(temp[0]);
        Serial.println(temp[1]);
        Serial.println(temp[2]);
        Serial.println(temp[3]);
        Serial.println("------- END -------");
        delay(1000);
    }
    Serial.println("Sensor system fine.");
}


#endif








 
