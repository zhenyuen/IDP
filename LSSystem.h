#ifndef SensorSystem_h
#define SensorSystem_h


#include <Arduino.h>
#include <Config.h>

class LSSystem {
  public:
    static void set_ls_b_high();
    static void set_ls_fm_high();
    static void set_ls_fl_high();
    static void set_ls_fr_high();
    static void set_ls_b_low();
    static void set_ls_fm_low();
    static void set_ls_fl_low();
    static void set_ls_fr_low();
    static bool get_ls_b();
    static bool get_ls_fm();
    static bool get_ls_fl();
    static bool get_ls_fr();
    static void test();

    
  private:
    static bool _ls_b;
    static bool _ls_fm;
    static bool _ls_fl;
    static bool _ls_fr;
};


bool LSSystem::_ls_b = false;
bool LSSystem::_ls_fm = false;
bool LSSystem::_ls_fl = false;
bool LSSystem::_ls_fr = false;



void LSSystem::set_ls_b_high() {
    _ls_b = true;
};

void LSSystem::set_ls_fm_high() {
    _ls_fm= true;
};

void LSSystem::set_ls_fl_high() {
    _ls_fl = true;
};

void LSSystem::set_ls_fr_high() {
    _ls_fr = true;
};

void LSSystem::set_ls_b_low() {
    _ls_b = false;
};
void LSSystem::set_ls_fm_low() {
    _ls_fm = false;
};

void LSSystem::set_ls_fl_low() {
    _ls_fl = false;
};
void LSSystem::set_ls_fr_low() {
    _ls_fr = false;
};

bool LSSystem::get_ls_b() {
    return _ls_b;
};

bool LSSystem::get_ls_fm() {
    return _ls_fm;
};

bool LSSystem::get_ls_fr() {
    return _ls_fr;
};

bool LSSystem::get_ls_fl() {
    return _ls_fl;
};

void LSSystem::test() {
    Serial.println("------- LINE SENSOR TEST -------");
    Serial.println("Place line sensors on black.");
    while (true) {
        if (!_ls_b && !_ls_fm && !_ls_fr && !_ls_fl) {
            break;
        }
    }
    Serial.println("Place line sensors on white.");
        while (true) {
        if (_ls_b && _ls_fm && _ls_fr && _ls_fl) {
            break;
        }
    }
    Serial.println("------- TEST PASSED -------");    
};
#endif








 
