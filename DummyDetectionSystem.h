#ifndef DummyDetectionSystem_h
#define DummyDetectionSystem_h

#include <Arduino.h>
#include "Config.h"
#include "IRReceiver.h"
#include "PhotoTransistor.h"


class DummyDetectionSystem
{
  public:
    void setup(IRReceiver* ir_receiver, PhotoTransistor* pt_m, PhotoTransistor* pt_l, PhotoTransistor* pt_r);
    // void identify_signal();
    uint8_t identify_dummy();
    void test();

  private:
    IRReceiver*_ir_receiver;
    PhotoTransistor* _pt_m;
    PhotoTransistor* _pt_l;
    PhotoTransistor* _pt_r;
};




void DummyDetectionSystem::setup(IRReceiver* ir_receiver, PhotoTransistor* pt_m, PhotoTransistor* pt_l, PhotoTransistor* pt_r) {
    _ir_receiver = ir_receiver;
    _pt_m = pt_m;
    _pt_l = pt_l;
    _pt_r = pt_r;
}


void DummyDetectionSystem::test() {
    for (int i = 0; i < 50; i++) {
        int score = identify_dummy();
        if (score == DUMMY_UNMODULATED) {
            Serial.println("UNMODULATED.");
        } else if (score == DUMMY_MODULATED){
            Serial.println("MODULATED.");
        } else {
            Serial.println("ALTERNATING.");
        }  
    }
    Serial.println("Dummy detection system fine.");
}


uint8_t DummyDetectionSystem::identify_dummy() {
    uint8_t scores[3] = {0, 0, 0};

    for (int i = 0; i < IR_HIST_LEN; i++) {
        uint8_t cycle_i = 0;
        uint8_t pulses = 0;

        for (int j = 0; j < IR_CYCLES; j++) {
            bool ir = _ir_receiver -> read();
            uint8_t analog = _pt_m -> read();

            while (analog < IR_ANALOG_THRESHOLD) {
                analog = _pt_m -> read();
                ir = _ir_receiver -> read();
            }
            if (ir) {
                pulses++;
            }
        }

        if (pulses >= 120) {
            scores[0]++;
        } else if (pulses >= 50 && pulses < 120) {
            scores[2]++;
        } else if (pulses < 50){
            scores[1]++;
        }
    }
    
    // Selecting ID of dummy based on score (Highest)
    uint8_t _max = scores[0];
    uint8_t type = DUMMY_MODULATED;
    if (scores[1] > _max) {
        _max = scores[1];
        type = DUMMY_UNMODULATED;
    }

    if (scores[2] > _max) {
        _max = scores[2];
        type = DUMMY_ALTERNATING;
    }
    return type;
}

#endif
