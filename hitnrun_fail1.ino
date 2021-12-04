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

volatile bool is_running = false;
bool dummy_collected = true;
bool dummy_detected = false;
bool location_changed = false;
bool location = false; // false - starting half, true - collection half
bool off_line_timer_start = false;
int8_t tilt = 0;
int8_t quad = -1; // -1 for middle line, 0 - left quad, 1 - right quad
uint8_t failed_attempts = 0;
uint8_t n_dummy_collected = 0;
uint8_t task_counter = -1;
uint8_t step_counter = 0;
uint8_t dummy_type = 0; // 1, 2 or 3
unsigned long timer;
unsigned long trigger_timer;
unsigned long off_line_timer;

// Debugging variables
int debug_task_counter;
bool debug_at_last_task;
int debug_task_counter_end;
int end_step_counter;

// ----------------------------------------------------------------------------------------------
void toggle_on();
void next_task(int task_id);
void update_location();
void follow_line(float steering_factor=0.8, uint8_t power=DEF_SPEED);
void rotate_180(bool cw);
void rotate_90(bool cw);
void reverse_to_line();
void exit_wbox();
void exit_rbbox(bool repeat);
void enter_rbbox(bool is_red);
void realign(bool cw, bool forward=true);
bool exit_box();
bool is_off_line();
bool locate_dummy(int quad);


// ----------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  
  // Pin modes
  pinMode(SWITCH_PIN, INPUT);
  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(A_LED_PIN, OUTPUT);
  
  AFMS.begin();

  // Setup LED indicators
  pinMode(A_LED_PIN, OUTPUT);
  digitalWrite(A_LED_PIN, HIGH);
  digitalWrite(G_LED_PIN, 0);
  digitalWrite(R_LED_PIN, 0);


  // Switch interrupt
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), toggle_on, RISING);

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


  if (DEBUG_MODE) {
    debug_task_counter = EXIT_RBBOX;
    //debug_task_counter_end = APPROACH_DUMMY;
    if (task_counter == HEAD_TO_COLLECTION_AREA) {
      step_counter = 0;
      end_step_counter = 2;
      location = 0;
    } else if (debug_task_counter == DETECT_DUMMY) {
      step_counter = 0;
      quad = -1;
      end_step_counter = 2;
      //n_dummy_collected = 0;
      n_dummy_collected = 0;
      //n_dummy_collected = 2;
    } else if (debug_task_counter == APPROACH_DUMMY) {
      step_counter = 0;
      end_step_counter = 1;
      claw_raiser.search();
    } else if (debug_task_counter == IDENTIFY_DUMMY) {
      
    } else if (debug_task_counter == COLLECT_DUMMY) {
      
    } else if (debug_task_counter == HEAD_TO_DROPOFF_POINT) {
      location = 1;
      step_counter = 0;
      //quad = -1;
      //quad = 0;
      quad = 1;
      end_step_counter = 4;
      //n_dummy_collected = 0;
      //n_dummy_collected = 1;
      n_dummy_collected = 2;
      //dummy_type = DUMMY_MODULATED;
      dummy_type = DUMMY_UNMODULATED;
      //dummy_type = DUMMY_ALTERNATING;
      
    } else if (debug_task_counter == DROP_DUMMY) {
      
    } else if (debug_task_counter == EXIT_WBOX) {
      step_counter = 0;
      end_step_counter = 2;
      n_dummy_collected = 0;
      //n_dummy_collected = 1;
      //n_dummy_collected = 2;
    } else if (debug_task_counter == EXIT_RBBOX) {
      step_counter = 0;
      end_step_counter = 1;
      n_dummy_collected = 1;
      //n_dummy_collected = 1;
      //n_dummy_collected = 2;
      dummy_type = DUMMY_UNMODULATED;
      //dummy_type = DUMMY_UNMODULATED;
      //dummy_type = DUMMY_ALTERNATING;
    } else if (debug_task_counter == RETURN_FROM_LOC_0) {
      location = 0;
      step_counter = 0;
      end_step_counter = 2;
    } else if (debug_task_counter == RETURN_FROM_LOC_1) {
      location = 1;
      step_counter = 0;
      end_step_counter = 5;
    }
    task_counter = debug_task_counter;
  } else {
    next_task(EXIT_BOX);
  }
  //location = 0;
}

void loop() {
  if (!is_running) {
    // Check if running
    drive_system.brake();
    return;
  }
  
  if (DEBUG_MODE) {
    Serial.println("------------------------- Loop start -------------------------");
    if (!debug_at_last_task && task_counter == debug_task_counter_end) debug_at_last_task = true;
    if (task_counter == HEAD_TO_COLLECTION_AREA) {
      Serial.println("step counter, location, tilt");
      Serial.println(step_counter);
      Serial.println(location);
      Serial.println(tilt);
    } else if (task_counter == DETECT_DUMMY) {
      Serial.println("step counter, failed attempts, quad");
      Serial.println(step_counter);
      Serial.println(failed_attempts);
      Serial.println(quad);
    } else if (task_counter == APPROACH_DUMMY) {
      Serial.println("step counter");
      Serial.println(step_counter);      
    } else if (task_counter == IDENTIFY_DUMMY) {
      Serial.println("dummy type");
      Serial.println(dummy_type);
    } else if (task_counter == COLLECT_DUMMY) {
      
    } else if (task_counter == HEAD_TO_DROPOFF_POINT) {
      Serial.println("step counter, quad, location, tilt, dummy type");
      Serial.println(step_counter);
      Serial.println(quad);
      Serial.println(location);
      Serial.println(tilt);
      Serial.println(dummy_type);
    } else if (task_counter == DROP_DUMMY) {
      
    } else if (task_counter == EXIT_WBOX) {
      Serial.println("step counter");
      Serial.println(step_counter);
    } else if (task_counter == EXIT_RBBOX) {
      Serial.println("step counter");
      Serial.println(step_counter);
    } else if (task_counter == RETURN_FROM_LOC_0) {
      Serial.println("step counter, location, tilt");
      Serial.println(step_counter);
      Serial.println(location);
      Serial.println(tilt);
    } else if (task_counter = RETURN_FROM_LOC_1) {
      Serial.println("step counter, location, tilt");
      Serial.println(step_counter);
      Serial.println(location);
      Serial.println(tilt);
    }
//    if (step_counter > end_step_counter) return;
//    if (task_counter != debug_task_counter) return;
  }
  
  update_location();
  
  // Execute tasks
  // --------------------------------------------------------------------------------
  if (task_counter == EXIT_BOX) { // COMPLETED
    Serial.println("Exiting box.");
    if (exit_box()) {
      drive_system.brake();
      dummy_collected = false;
      dummy_detected = false;
      next_task(HEAD_TO_COLLECTION_AREA);
    }

  // --------------------------------------------------------------------------------
  } else if (task_counter == HEAD_TO_COLLECTION_AREA) { // UNCOMPLETE
    Serial.println("Head to collection area.");
    if (step_counter == 0) {
      
      Serial.println("Step 0");
      if (acc.get_state() == ASCEND_STATE) {
        follow_line(0.9, DEF_SPEED);
      } else {
        follow_line(0.6, DEF_SPEED);
      }
      if (location == 1 && tilt == 0) step_counter++;
    } else if (step_counter == 1) {
      Serial.println("Step 1");
      timer = millis();
      while(true) {
        if (millis() - timer > TIME_FROM_SLOPE && tilt == 0) break;
        follow_line(0.6, DEF_SPEED);
      }
      // realign(true); // Can realign after slope.
      step_counter++;
    } else if (step_counter == 2) {
      Serial.println("Step 2");
      next_task(DETECT_DUMMY);
      interrupts();
    }


  // --------------------------------------------------------------------------------
  } else if (task_counter == DETECT_DUMMY) { // UNCOMPLETE
    Serial.println("Detect dummy.");
    if (step_counter == 0) {
      Serial.println("Step 0");
      // Setup
      claw_raiser.search();
      dummy_collected = false;
      step_counter++;
    } else if (step_counter == 1) {
      Serial.println("Step 1");
      if (n_dummy_collected == 0) {
        // First dummy on white line
        quad = -1;
        step_counter++;
      } else if (n_dummy_collected == 1) {
        quad = 0;
        step_counter++;
      } else if (n_dummy_collected == 2) {
        quad = 1;
        step_counter++;
      }
    } else if (step_counter == 2) {
      Serial.println("Step 2");
      bool detected = locate_dummy(quad);
      if (detected) {
        next_task(APPROACH_DUMMY);
      } else {
        // Failed to locate dummy
        failed_attempts++;
        if (failed_attempts == FAILURE_LIMIT) {
          locate_dummy((quad + failed_attempts) % 2);
          next_task(COMPLETED);
        } else {
          next_task(APPROACH_DUMMY);
        }
      }
    }
 
  // --------------------------------------------------------------------------------
  } else if (task_counter == APPROACH_DUMMY) { // UNCOMPLETE
    Serial.println("Approach dummy");
    if (step_counter == 0) {
      Serial.println("Step 0");
      approach_dummy();
      next_task(IDENTIFY_DUMMY);
    } 

  // --------------------------------------------------------------------------------
  } else if (task_counter == IDENTIFY_DUMMY) { // UNCOMPLETE
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
    pinMode(A_LED_PIN, OUTPUT);
    digitalWrite(A_LED_PIN, HIGH);
    delay(5000);
    pinMode(A_LED_PIN, INPUT);
    next_task(COLLECT_DUMMY);

  // --------------------------------------------------------------------------------  
  } else if (task_counter == COLLECT_DUMMY) { // UNCOMPLETE
    // Collect dummy
    drive_system.forward(150);
    delay(250);
    drive_system.brake();
    claw_raiser.down();
    delay(200);
    claw.close();
    delay(200);
    claw_raiser.up();
    dummy_collected = true;

    next_task(HEAD_TO_DROPOFF_POINT);

  // --------------------------------------------------------------------------------
  } else if (task_counter == HEAD_TO_DROPOFF_POINT) { // UNCOMPLETE
    Serial.println("Head to dropoff point.");

    if (step_counter == 0) {
      Serial.println("Step 0");
      if (n_dummy_collected == 0) {
        if (dummy_type != DUMMY_MODULATED) {
          rotate_180(true);
          realign(false);
        }
      } else {
        if (quad == 0) {
          drive_system.steer(DEF_SPEED, true, 0.6);
          delay(1000);
        } else {
          drive_system.steer(DEF_SPEED, false, 0.6);
          delay(1000);
        }
        reverse_to_line();
        drive_system.brake();
        if (dummy_type == DUMMY_MODULATED) {
          if (quad == 1) {
            realign(true);
          } else if (quad == 0) {
            realign(false);
          }
        } else {
          if (quad == 1) {
            realign(false);
          } else if (quad == 0) {
            realign(true);
          }
        }
      }
      step_counter++;
    } else if (step_counter == 1) {
      Serial.println("Step 1");
      if (dummy_type == DUMMY_MODULATED) {
        // Heading to white box.
        follow_line(0.6, DEF_SPEED);
        if (junction_reached()) {
          next_task(DROP_DUMMY);
        }
      } else {
        // Heading to blue/red box.
        if (acc.get_state() == ASCEND_STATE) {
          follow_line(0.9, DEF_SPEED);
        } else {
          follow_line(0.6, DEF_SPEED);
        }
        //if(location == 0 && tilt == 0) step_counter++;
        if (junction_reached()) step_counter = 4;
      }
    } else if (step_counter == 2) {
      Serial.println("Step 2");
      timer = millis();
      while(true) {
        if (millis() - timer > TIME_FROM_SLOPE && tilt == 0) break;
        follow_line(0.6, DEF_SPEED);
      }
      //realign(true); // Can realign after slope if necessary;
      step_counter++;
    } else if (step_counter == 3) {
      Serial.println("Step 3");
      follow_line(0.6, 200);
      if (junction_reached()) step_counter++; // Reached first junction
    } else if (step_counter == 4) {
      Serial.println("Step 4");
      // Move forward for a fixed interval
      timer = millis();
      while(millis() - timer < 1000) {
        follow_line(0.9, 150);
      }
      enter_rbbox(dummy_type == DUMMY_UNMODULATED);
      next_task(DROP_DUMMY);
    }


  // --------------------------------------------------------------------------------
  } else if (task_counter == DROP_DUMMY) { // UNCOMPLETE
    Serial.println("Drop dummy.");
    claw_raiser.down();
    delay(200);
    claw.open();
    delay(200);
    claw_raiser.up();
    drive_system.backward(150);
    delay(100);
    n_dummy_collected++;
    dummy_collected = false;
    digitalWrite(G_LED_PIN, 0);
    digitalWrite(R_LED_PIN, 0);
    if (dummy_type == DUMMY_MODULATED) {
      next_task(EXIT_WBOX);
    } else  {
      next_task(EXIT_RBBOX);
    }
    
    
  // --------------------------------------------------------------------------------  
  } else if (task_counter == EXIT_WBOX) { // UNCOMPLETE
    Serial.println("Exiting white box");
    if (step_counter == 0) {
      Serial.println("Step 0");
      exit_wbox();
      step_counter++;
    } else if (step_counter == 1) {
      Serial.println("Step 1");
      if (n_dummy_collected == N_DUMMIES) {
        next_task(RETURN_FROM_LOC_1);
      } else {
        // Stop before slope
        timer = millis();
        while(millis() - timer < 3000) {
          follow_line(0.6, DEF_SPEED);
        }
        drive_system.brake();
        step_counter++;
      }
    } else if (step_counter == 2) {
      Serial.println("Step 2");
      rotate_180(true);
      realign(false);
      next_task(DETECT_DUMMY);
    }

  // --------------------------------------------------------------------------------   
  } else if (task_counter == EXIT_RBBOX) { // UNCOMPLETE
    Serial.println("Exiting RB box");
    if (step_counter == 0) {
      Serial.println("Step 0");
      if (dummy_type == DUMMY_UNMODULATED) {
        exit_rbbox(n_dummy_collected != N_DUMMIES);
      } else if (dummy_type == DUMMY_ALTERNATING) {
        exit_rbbox(n_dummy_collected == N_DUMMIES);
      }
      step_counter++;
    } else if (step_counter == 1) {
      Serial.println("Step 1");
      if (n_dummy_collected == N_DUMMIES) {
        next_task(RETURN_FROM_LOC_0);
      } else {
        next_task(HEAD_TO_COLLECTION_AREA);
      }
    }


  // --------------------------------------------------------------------------------
  } else if (task_counter == COMPLETED) { // UNCOMPLETE
    Serial.println("Completed");
    drive_system.brake();
    pinMode(A_LED_PIN, OUTPUT);
    digitalWrite(A_LED_PIN, HIGH);
    digitalWrite(G_LED_PIN, 0);
    digitalWrite(R_LED_PIN, 0);
    is_running = false;
    return;


  // --------------------------------------------------------------------------------
  } else if (task_counter == RETURN_FROM_LOC_0) { // UNCOMPLETE
    Serial.println("Returning to start from location 0");
    if (step_counter == 0) {
      Serial.println("Step 0");
      follow_line(0.6, DEF_SPEED);
      if (junction_reached()) {
        step_counter++; // Reached first junction
        delay(1000); // Make sure robot crossed first junction to prevent instant trigger
      }
    } else if (step_counter == 1) {
      Serial.println("Step 1");
      follow_line(0.6, DEF_SPEED);
      if (junction_reached()) step_counter++; // Reached second junction
    } else if (step_counter == 2) {
      Serial.println("Step 2");
      timer = millis();
      while (millis() - timer < TIME_MOVE_INTO_BOX) {
        if (is_off_line()) {
          next_task(COMPLETED);
        }
        drive_system.forward(DEF_SPEED); // Move into box
      }
      drive_system.brake();
      next_task(COMPLETED);
    }

    if (is_off_line()) next_task(COMPLETED);

    
  // --------------------------------------------------------------------------------
  } else if (task_counter == RETURN_FROM_LOC_1) { // UNCOMPLETE
    Serial.println("Returning to start from location 1");
    if (step_counter == 0) {
      Serial.println("Step 0");
      reverse_to_line();
      drive_system.brake();
      rotate_180(true);
      realign(false);
      step_counter++;
    } else if (step_counter == 1) {
      Serial.println("Step 1");
      if (acc.get_state() == ASCEND_STATE) {
        follow_line(0.9, DEF_SPEED);
      } else {
        follow_line(0.6, DEF_SPEED);
      }
      if (location == 0 && tilt == 0) step_counter++;
    } else if (step_counter == 2) {
      Serial.println("Step 2");
      timer = millis();
      while(true) {
        if (millis() - timer > TIME_FROM_SLOPE && tilt == 0) break;
        follow_line(0.6, DEF_SPEED);
      }
      step_counter++;
    } else if (step_counter == 3) {
      Serial.println("Step 3");
      follow_line(0.6, DEF_SPEED);
      if (junction_reached()) {
        step_counter++; // Reached first junction
        delay(1000); // Make sure robot crossed first junction to prevent instant trigger
      }
    } else if (step_counter == 4) {
      Serial.println("Step 4");
      follow_line(0.6, DEF_SPEED);
      if (junction_reached()) step_counter++; // Reached second junction
    } else if (step_counter == 5) {
      Serial.println("Step 5");
      timer = millis();
      while (millis() - timer < TIME_MOVE_INTO_BOX) {
        if (is_off_line()) {
          next_task(COMPLETED);
          return;
        }
        drive_system.forward(DEF_SPEED); // Move into box
      }
      drive_system.brake();
      next_task(COMPLETED);
    }
    if (is_off_line()) next_task(COMPLETED);
  }
}


bool is_off_line() {
  bool ls_b = !digitalRead(LS_B_PIN);
  bool ls_fm = !digitalRead(LS_FM_PIN);
  bool ls_fl = !digitalRead(LS_FL_PIN);
  bool ls_fr = !digitalRead(LS_FR_PIN);
  if (!off_line_timer_start) {
    if (ls_b || ls_fm || ls_fl || ls_fr) return false;
    off_line_timer_start = true;
    off_line_timer = millis();
  } else {
    if (ls_b || ls_fm || ls_fl || ls_fr) {
      off_line_timer_start = false;
    } else {
      if (millis() - off_line_timer > OFF_LINE_THRESHOLD) {
        off_line_timer_start = false;
        return true;
      }
    } 
  }
  return false;
}


void update_location() {
  acc.update_state();

  if (!location_changed && acc.get_state() == DESCEND_STATE) {
    location_changed = true;
  } else if (location_changed && acc.get_state() == NORMAL_STATE){
    location = !location;
    location_changed = false;
  }
}


bool exit_box() {
  bool ls_b = !digitalRead(LS_B_PIN);
  bool ls_fm = !digitalRead(LS_FM_PIN);
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


void follow_line(float steering_factor, uint8_t power) {
  bool ls_b = !digitalRead(LS_B_PIN);
  bool ls_fm = !digitalRead(LS_FM_PIN);
  bool ls_fl = !digitalRead(LS_FL_PIN);
  bool ls_fr = !digitalRead(LS_FR_PIN);
  if (tilt == 0) {
    drive_system.forward(power);
    if (ls_fl && !ls_fm && !ls_fr) {
      drive_system.steer(power, true);
      tilt = 1;
    } else if (ls_fr && !ls_fm && !ls_fl) {
      drive_system.steer(power, false);
      tilt = -1;
    }
  } else {
    if (ls_b) {
      if (tilt == 1) {
        drive_system.steer(power, true, steering_factor);
      } else {
        drive_system.steer(power, false, steering_factor);
      }
      
    } else if (!ls_b) {
      if (tilt == 1) {
        drive_system.steer(power, true, steering_factor - 0.1);
      } else {
        drive_system.steer(power, false, steering_factor - 0.1);
      }
    }
    
    if (ls_fm && !ls_fl && !ls_fr) {
      tilt = 0;
    }
  }
}


void realign(bool cw, bool forward) {
  bool ls_b = !digitalRead(LS_B_PIN);
  bool ls_fm = !digitalRead(LS_FM_PIN);
  
  if (ls_fm && ls_b) return;
  
  if (!ls_b) {
    bool on_line = false;
    if (forward) {
      drive_system.forward(DEF_SPEED);
    } else {
      drive_system.backward(DEF_SPEED);
    }
    timer = millis();
    while (millis() - timer < TIME_REALIGN_FORWARD) {
      ls_b = !digitalRead(LS_B_PIN);
      if (ls_b) {
        drive_system.brake();
        on_line = true;
        break;
      }
    }

    if (!on_line) {
      if (forward) {
        drive_system.backward(DEF_SPEED);
      } else {
        drive_system.forward(DEF_SPEED);
      }
      timer = millis();
      while (millis() - timer < TIME_REALIGN_BACKWARD) {
        ls_b = !digitalRead(LS_B_PIN);
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
  while(!ls_fm) {
    ls_fm = !digitalRead(LS_FM_PIN);;
  }
  drive_system.brake();
  return;
}


void reverse_to_line() {
  bool ls_b = !digitalRead(LS_B_PIN);
  drive_system.backward(DEF_SPEED);
  while(!ls_b) {
    ls_b = !digitalRead(LS_B_PIN);
  }
  drive_system.brake();
  return;
}


bool locate_dummy(int quad) {
  bool ls_fm = !digitalRead(LS_FM_PIN);
  if (quad == 0) {
    drive_system.rotate(100, true);
    delay(1500);
    while(!ls_fm) {
      ls_fm = !digitalRead(LS_FM_PIN);
      if (pt_wide.read() > 70) {
        drive_system.brake();
        return true;
      }
    }
    return false;
  } else if (quad == 1) {
    drive_system.rotate(100, false);
    delay(1500);
  
    while(!ls_fm) {
      ls_fm = !digitalRead(LS_FM_PIN);
      if (pt_wide.read() > 70) {
        drive_system.brake();
        return true;
      }
    }
    return false;
  } else if (quad == -1) {
    drive_system.rotate(100, true);
    timer = millis();
    while(true) {
      ls_fm = !digitalRead(LS_FM_PIN);
      if (pt_wide.read() > 150 || millis() - timer > 1500) {
        drive_system.brake();
        return true;
      }
    }
    drive_system.rotate(100, false);
    timer = millis();
    while(true) {
      ls_fm = !digitalRead(LS_FM_PIN);
      if (pt_wide.read() > 150 || millis() - timer > 3000) {
        drive_system.brake();
        return true;
      }
    }
    return false;
  }
}

bool junction_reached() {
  bool ls_fm = !digitalRead(LS_FM_PIN);
  bool ls_fl = !digitalRead(LS_FL_PIN);
  bool ls_fr = !digitalRead(LS_FR_PIN);
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
  bool ls_fm = !digitalRead(LS_FM_PIN);

  drive_system.rotate(DEF_SPEED, cw);
  delay(1000);
  
  ls_fm = !digitalRead(LS_FM_PIN);
  while(!ls_fm) {
    ls_fm = !digitalRead(LS_FM_PIN);
  }
  drive_system.brake();
  return;
}


void enter_rbbox(bool is_red) {
  rotate_90(is_red);
  drive_system.steer(150, false, 0.6);
  delay(200);
  drive_system.forward(150);
  delay(200);
  drive_system.brake();
}


void exit_wbox() {
  drive_system.backward(DEF_SPEED);
  delay(1000);
  rotate_180(true);
}

void exit_rbbox(bool repeat) {
  // MARK IMPORTANT
  drive_system.backward(DEF_SPEED);
  delay(500); // Make sure back line sensor crosses junction line
  rotate_90(repeat);
}

void next_task(int task_number) {
  drive_system.brake();
  failed_attempts = 0;
  step_counter = 0;
  task_counter = task_number;
  timer = millis();
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

float ultrasound_read() {
  float score = 0;
  for (int i = 0; i < 10; i++) {
    score += us.read();
    delay(20);
  }
  return score / 10;
}

void approach_dummy() {
  // MARK IMPORTANT
  int score = 0;
  int threshold = 100;
  int sig_increment = 10;
  bool cw = false;
  unsigned long realign_timer = millis();

  while(true) {
    if (DEBUG_MODE) {
      Serial.println("Moving towards dummy");
      Serial.println(score);
    }
    if (score > 0) break;
    drive_system.steer(200, false, 0.9);
    if (millis() - realign_timer > TIME_REALIGN_TO_DUMMY_INTERVAL) {
      // Realign by considering analog signal from dummy
      if (DEBUG_MODE) {
        Serial.println("Realigning to dummy, threshold");
        Serial.println(threshold);
      }
      drive_system.brake();
      drive_system.rotate(80, cw);
      timer = millis();
      while (millis() - timer < TIME_REALIGN_TO_DUMMY_ACW) {
        if (pt_narrow.read() > threshold) {
          delay(TIME_REALIGN_TO_DUMMY_DELAY);
          drive_system.brake();
          float distance = ultrasound_read();
          if (DEBUG_MODE) Serial.println(distance);
          if (distance < 5) score++;
          break;
        }
      }
      realign_timer = millis(); 
      threshold += sig_increment;
    }
  }

}
