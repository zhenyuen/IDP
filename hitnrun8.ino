#include <Arduino.h>
#include <Adafruit_MotorShield.h>

#include "Config.h"
#include "Ultrasound.h"
#include "IRReceiver.h"
#include "PhotoTransistor.h"
#include "DriveSystem.h"
#include "DummyDetectionSystem.h"
#include "Accelerometer.h"
#include "ClawRaiser.h"
#include "Claw.h"


// ----------------------------------------------------------------------------------------------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Ultrasound us;

IRReceiver ir_r;
PhotoTransistor pt_narrow;
PhotoTransistor pt_wide;
Accelerometer acc;

DriveSystem drive_system;
DummyDetectionSystem dd_system;

Claw claw;
ClawRaiser claw_raiser;

volatile bool ls_b, ls_fm, ls_fl, ls_fr;
bool debug_mode = false;
volatile bool is_running = false;
bool dummy_collected = false;
bool dummy_detected = false;
bool location_changed = false;
bool ls_b_timer;
bool ls_fm_timer;
bool ls_fl_timer;
bool ls_fr_timer;
int tilt = 0;
uint8_t quad = 0; // 0 for middle line, 1 - left quad, 2 - right quad
uint8_t n_dummy_collected = 0;
uint8_t location = 0; // 0 - starting half, 1 - collection half 
uint8_t task_counter = APPROACH_DUMMY;
uint8_t step_counter = 0;
uint8_t dummy_type = DUMMY_ALTERNATING; // 1, 2 or 3
unsigned long timer;
unsigned long trigger_timer; 


// ----------------------------------------------------------------------------------------------
void trigger_leds();
void follow_line(float steering_factor=0.8);
void rotate_180(bool cw);
void rotate_90(bool cw);
void reverse_to_line();
void exit_rbbox(bool repeat);
void enter_rbbox(bool is_red);
bool exit_box();

// ----------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  
  // Pin modes
  pinMode(SWITCH_PIN, INPUT);
  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(A_LED_PIN, OUTPUT);

  // Setup components and systems
  ir_r.setup(IR_PIN);
  pt_narrow.setup(PT_NARROW_PIN);
  pt_wide.setup(PT_WIDE_PIN);
  acc.setup(ACC_PIN);
  us.setup(US_ECHO_PIN, US_TRIG_PIN);
  claw.setup(CLAW_PIN);
  claw_raiser.setup(CLAW_RAISER_PIN);

  dd_system.setup(&ir_r, &pt_narrow, nullptr, nullptr);
  drive_system.setup(&AFMS, DC_M_L_PIN, DC_M_R_PIN);
  trigger_timer = millis();
  ls_b_timer = millis();
  ls_fm_timer = millis();
  ls_fl_timer = millis();
  ls_fr_timer = millis();
  
  AFMS.begin();

  // Setup LED indicators
  pinMode(A_LED_PIN, OUTPUT);
  digitalWrite(A_LED_PIN, HIGH);
  digitalWrite(G_LED_PIN, 0);
  digitalWrite(R_LED_PIN, 0);

  // Switch interrupt
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), toggle_on, RISING);
  // Line sensor interrupts
  attachInterrupt(digitalPinToInterrupt(LS_B_PIN), toggle_ls_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LS_FM_PIN), toggle_ls_fm, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LS_FL_PIN), toggle_ls_fl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LS_FR_PIN), toggle_ls_fr, CHANGE);
  ls_b = !digitalRead(LS_B_PIN);
  ls_fm = !digitalRead(LS_FM_PIN);
  ls_fl = !digitalRead(LS_FL_PIN);
  ls_fr = !digitalRead(LS_FR_PIN);
  Serial.println("Hi");
  if (debug_mode) {
//     while(!is_running);
//    Serial.println("------- LINE SENSOR TEST -------");
//    Serial.println("Place line sensors on black.");
//    while (true) {
//      Serial.println(ls_b);
//      Serial.println(ls_fm);
//      Serial.println(ls_fl);
//      Serial.println(ls_fr);
//      Serial.println("---------");
//      delay(500);
//        if (!ls_b && !ls_fm && !ls_fr && !ls_fl) {
//            break;
//        }
//    }
//    Serial.println("Place line sensors on white.");
//        while (true) {
//      Serial.println(ls_b);
//      Serial.println(ls_fm);
//      Serial.println(ls_fl);
//      Serial.println(ls_fr);
//      Serial.println("---------");
//      delay(500);
//        if (ls_b && ls_fm && ls_fr && ls_fl) {
//            break;
//        }
//    }
//    Serial.println("------- TEST PASSED -------"); 
      
    //drive_system.test();
    //dd_system.test();
    //us.test();
    //acc.test();
    //rotate_90(true);
    //realign(false);
  }
}



void loop() {
  if (!is_running) {
    drive_system.brake();
    return;
  }
  
  if (debug_mode) {
    //Serial.println(location);
    acc.update_state();
    Serial.println(acc.get_ay());
    Serial.println(acc.get_state());
    follow_line(0.8);
    delay(100);
    return;
  }
  // Check if running
  update_location();
  
  // Execute tasks
  if (task_counter == EXIT_BOX) {
    Serial.println("Exiting box.");
    if (exit_box()) {
      claw_raiser.up();
      claw.open();
      dummy_collected = false;
      dummy_detected = false;
      next_task(HEAD_TO_COLLECTION_AREA);
    }


  } else if (task_counter == HEAD_TO_COLLECTION_AREA) {
    Serial.println("Head to collection area.");

    if (location == 1 && tilt == 0) {
      timer = millis();
      while(millis() - timer < 1000) {
        follow_line(0.9);
      }
      delay(500);
      claw_raiser.search();
      realign(true);
      next_task(DETECT_DUMMY);
    }
    if (acc.get_state() == ASCEND_STATE) {
      follow_line(0.9);
    } else {
      follow_line(0.6);
    }


  } else if (task_counter == DETECT_DUMMY) {
    Serial.println("Detect dummy."); 

    if (n_dummy_collected == 0) {
      next_task(APPROACH_DUMMY);
    } else {
      if (locate_dummy()) {
        next_task(APPROACH_DUMMY);
      } else {
        Serial.println("Failed to locate dummy");
        next_task(COMPLETED);
      }
    }
    
    
  } else if (task_counter == APPROACH_DUMMY) {
    Serial.println("Approach dummy");
    // Move forward in straight line until dummy reached.

    if (step_counter == 0) {
      if (us.read() < 8) {
        drive_system.brake();
        delay(5000);
        while(us.read() > 4) {
          drive_system.forward(150);
        }
        drive_system.brake();
        step_counter++;
      } else {
        if (n_dummy_collected == 0) {
          follow_line();
        } else {
          drive_system.forward(DEF_SPEED);
        }
      }
    } else if (step_counter == 1) {
      // Ensure dummy is in line of sight
      drive_system.rotate(80, true);
      timer = millis();
      while (millis() - timer < 2000) {
        if (pt_narrow.read() > 200) {
          next_task(IDENTIFY_DUMMY);
          return;  
        }
      }
      
      drive_system.rotate(80, false);
      timer = millis();
      while (millis() - timer < 4000) {
        if (pt_narrow.read() > 200) {
          next_task(IDENTIFY_DUMMY);
          return;  
        }
      }
    }

   
  } else if (task_counter == IDENTIFY_DUMMY) {
    Serial.println("Identify dummy.");
    dummy_type = dd_system.identify_dummy();
    
    if (dummy_type == DUMMY_MODULATED) {
      digitalWrite(R_LED_PIN, 1);
      digitalWrite(G_LED_PIN, 1);
    } else if (dummy_type == DUMMY_UNMODULATED) {
      digitalWrite(R_LED_PIN, 1);
    } else if (dummy_type == DUMMY_ALTERNATING) {
      digitalWrite(G_LED_PIN, 1);
    }

    next_task(COLLECT_DUMMY);


    
  } else if (task_counter == COLLECT_DUMMY) {
    // Collect dummy
    claw_raiser.down();
    delay(200);
    claw.close();
    delay(200);
    claw_raiser.up();
    dummy_collected = true;
    next_task(HEAD_TO_DROPOFF_POINT);
    
  } else if (task_counter == HEAD_TO_DROPOFF_POINT) {
    return;
    Serial.println("Head to dropoff point.");

    if (step_counter == 0) {
      if (n_dummy_collected != 0) reverse_to_line();
      step_counter++;
    } else if (step_counter == 1) {
      // Line reached
      if (n_dummy_collected == 0) {
        if (dummy_type == DUMMY_MODULATED) {
          follow_line(0.9);
          if (junction_reached()) {
            next_task(DROP_DUMMY);
            return;
          }
        } else {
          rotate_180(true);
          realign(true);
          step_counter++;
        }
      } else {
        if (dummy_type == DUMMY_MODULATED) {
          if (quad == 1) {
            realign(true);
          } else if (quad == 2) {
            realign(false);
          }
        } else {
          if (quad == 1) {
            realign(false);
          } else if (quad == 2) {
            realign(true);
          }
        }
        step_counter++;
      }
    } else if (step_counter == 2) {
      if (junction_reached()) {
        // Move forward for a fixed interval
        timer = millis();
        while(millis() - timer < 500) {
          follow_line(0.9);
        }
        enter_rbbox(dummy_type == DUMMY_UNMODULATED);
        next_task(DROP_DUMMY);
      }
      if (acc.get_state() == ASCEND_STATE) {
        follow_line(0.9);
      } else {
        follow_line(0.6);
      }
    }

    
  } else if (task_counter == DROP_DUMMY) {
    return;
    Serial.println("Drop dummy.");
    claw_raiser.down();
    delay(200);
    claw.open();
    delay(200);
    claw_raiser.up();
    n_dummy_collected++;
    dummy_collected = false;
    digitalWrite(G_LED_PIN, 0);
    digitalWrite(R_LED_PIN, 0);
    if (dummy_type == DUMMY_MODULATED) {
      next_task(EXIT_WBOX);
    } else  {
      next_task(EXIT_RBBOX);
    }
    
    
  } else if (task_counter == EXIT_WBOX) {
    return;
    Serial.println("Exiting whitebox");
    if (step_counter == 0) {
      exit_wbox();
      step_counter++;
    }
    
    if (n_dummy_collected == 3) {
      // Return to starting box;
      if (junction_reached()) {
        timer = millis();
        while(millis() - timer < 2000) {
          follow_line(0.9);
          if (junction_reached()) break;
        }
        // Reached starting box           
        drive_system.forward(DEF_SPEED);
        delay(2000);
        Serial.println("Completed");
        is_running = false;
        next_task(COMPLETED);
        return;
      }
      if (acc.get_state() == ASCEND_STATE) {
        follow_line(0.9);
      } else {
        follow_line(0.6);
      }
      
    } else {
      if (step_counter == 1) {
        timer = millis();
        // Stop before slope
        while(millis() - timer < 3000) {
          follow_line();
        }
        drive_system.brake();
        step_counter++;

      } else if (step_counter == 2) {
        // Rotate 180
        rotate_180(true);
        realign(true);
        next_task(DETECT_DUMMY);
      }
    }

    
  } else if (task_counter == EXIT_RBBOX) {
    return;
    if (step_counter == 0) {
      if (dummy_type == DUMMY_UNMODULATED) {
        exit_rbbox(n_dummy_collected != 3);
        step_counter++;
      } else if (dummy_type == DUMMY_ALTERNATING) {
        exit_rbbox(n_dummy_collected == 3);
        step_counter++;
      }
    } else if (step_counter == 1) {
      if (n_dummy_collected == 3) {
        timer = millis();
        while (millis() - timer < 3000) {
          follow_line(0.9);
        }
        while(!junction_reached()) {
          follow_line(0.9);
        }
        drive_system.forward(DEF_SPEED);
        delay(2000);
        Serial.println("Completed");
        is_running = false;
        next_task(COMPLETED);
        return;
        
      } else {
        next_task(HEAD_TO_COLLECTION_AREA);
        return;
      }
    }
  } 
}


void update_location() {
  acc.update_state();
  if (!location_changed && acc.get_state() == DESCEND_STATE) {
    location++;
    location %= 2;
    location_changed = true;
  } else if (location_changed && acc.get_state() == NORMAL_STATE){
    location_changed = false;
  }
}


bool exit_box() {
  if (step_counter == 0) {
    drive_system.steer(DEF_SPEED, false);
    if (ls_b) {
        drive_system.brake();
        step_counter++;
    }
  } else if (step_counter == 1) {
    drive_system.rotate(150, false);
    if (ls_fm) {
      return true;
    }
  }
  return false;
}


void follow_line(float steering_factor) {
  if (tilt == 0) {
    drive_system.forward(DEF_SPEED);
    if (ls_fl && !ls_fm && !ls_fr) {
      drive_system.steer(DEF_SPEED, true);
      tilt = 1;
    } else if (ls_fr && !ls_fm && !ls_fl) {
      drive_system.steer(DEF_SPEED, false);
      tilt = -1;
    }
  } else {
    if (ls_b) {
      if (tilt == 1) {
        drive_system.steer(DEF_SPEED, true, steering_factor);
      } else {
        drive_system.steer(DEF_SPEED, false, steering_factor);
      }
      
    } else if (!ls_b) {
      if (tilt == 1) {
        drive_system.steer(DEF_SPEED, true, steering_factor - 0.1);
      } else {
        drive_system.steer(DEF_SPEED, false, steering_factor - 0.1);
      }
    }
    
    if (ls_fm && !ls_fl && !ls_fr) {
      tilt = 0;
    }
  }
}


void realign(bool cw) {
  if (ls_fm && ls_b) return;
  
  if (!ls_b) {
    bool on_line = false;
    drive_system.forward(DEF_SPEED);
    timer = millis();
    while (millis() - timer < 2500) {
      if (ls_b) {
        drive_system.brake();
        on_line = true;
        break;
      }
    }

    if (!on_line) {
      drive_system.backward(DEF_SPEED);
      timer = millis();
      while (millis() - timer < 5000) {
        if (ls_b) {
          drive_system.brake();
          on_line = true;
          break;
        } 
      }
    }
  }
  // Realign to line by looking at front line sensor
  drive_system.rotate(DEF_SPEED, cw);
  while(!ls_fm);
  drive_system.brake();
  return;
}


void reverse_to_line() {
  drive_system.backward(DEF_SPEED);
  while(!ls_b);
  drive_system.brake();
  return;
}


bool locate_dummy() {
  bool dummy_found = false;

  quad = 1;
  drive_system.rotate(100, false);
  delay(500);

  while(!ls_fm) {
    if (pt_wide.read() > 700) {
      drive_system.brake();
      dummy_found = true;
    }
  }

  if (dummy_found) return true;
  
  quad = 2;
  drive_system.rotate(100, false);
  delay(500);

  while(!ls_fm) {
    if (pt_wide.read() > 700) {
      drive_system.brake();
      dummy_found = true;
    }
  }

  if (dummy_found) return true;
  return false;
}

bool junction_reached() {
  if (ls_fm && ls_fl && ls_fr) {
    return true;
  }
  return false;
}


void rotate_90(bool cw) {
  drive_system.rotate(DEF_SPEED, cw);
  delay(2000);
  drive_system.brake();
  return;
}

void rotate_180(bool cw) {
  drive_system.rotate(DEF_SPEED, cw);
  delay(1000);

  while(!ls_fm);
  drive_system.brake();
  return;
}

void enter_rbbox(bool is_red) {
  rotate_90(is_red);
  drive_system.steer(DEF_SPEED, false, 0.6);
  delay(500);
  realign(is_red);
  drive_system.forward(DEF_SPEED);
  while (!ls_fm && !ls_fl && !ls_fr);
  drive_system.brake();
}


void exit_wbox() {
  drive_system.backward(DEF_SPEED);
  delay(400);
  rotate_180(true);
  realign(true);
}

void exit_rbbox(bool repeat) {
  drive_system.backward(DEF_SPEED);
  delay(400);
  rotate_180(true);
  realign(true);
}

void next_task(int task_number) {
  drive_system.brake();
  step_counter = 0;
  task_counter = task_number;
  timer = millis();
}

void toggle_ls_b() {
  if (millis() - ls_b_timer < 500) return;
  ls_b_timer = millis();
  ls_b = !digitalRead(LS_B_PIN);
}

void toggle_ls_fm() {
  if (millis() - ls_fm_timer < 500) return;
  ls_fm_timer = millis();
  ls_fm = !digitalRead(LS_FM_PIN);
}

void toggle_ls_fl() {
  if (millis() - ls_fl_timer < 500) return;
  ls_fl_timer = millis();
  ls_fl = !digitalRead(LS_FL_PIN);
}

void toggle_ls_fr() {
  if (millis() - ls_fr_timer < 500) return;
  ls_fr_timer = millis();
  ls_fr = !digitalRead(LS_FR_PIN);
}

void toggle_on() {
  if (millis() - trigger_timer < 250) return;
  trigger_timer = millis();
  
  if (is_running) {
    Serial.println("Stopped");

    is_running = false;
    pinMode(A_LED_PIN, OUTPUT);
    digitalWrite(A_LED_PIN, HIGH);
    digitalWrite(G_LED_PIN, 0);
    digitalWrite(R_LED_PIN, 0);
  } else {
    Serial.println("Running");
    is_running = true;
    pinMode(A_LED_PIN, INPUT);
  }
  //delay(500);
}
