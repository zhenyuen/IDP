#include <Arduino.h>
#include <Adafruit_MotorShield.h>

#include "Config.h"
#include "LineSensor.h"
#include "Ultrasound.h"
#include "IRReceiver.h"
#include "PhotoTransistor.h"
#include "SensorSystem.h"
#include "DriveSystem.h"
#include "DummyDetectionSystem.h"
#include "Accelerometer.h"
#include "ClawRaiser.h"
#include "Claw.h"


// ----------------------------------------------------------------------------------------------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

LineSensor ls_b;
LineSensor ls_fm;
LineSensor ls_fl;
LineSensor ls_fr;
Ultrasound us;

IRReceiver ir_r;
PhotoTransistor pt_m;
Accelerometer acc;

SensorSystem sensor_system;
DriveSystem drive_system;
DummyDetectionSystem dd_system;

Claw claw;
ClawRaiser claw_raiser;


bool is_running = false;
bool dummy_collected = false;
bool dummy_detected = false;
bool location_changed = false;
int tilt = 0;
uint8_t quad = 0; // 0 for middle line, 1 - left quad, 2 - right quad
uint8_t n_dummy_collected = 0;
uint8_t location = 0; // 0 - starting half, 1 - collection half 
uint8_t task_counter = 0;
uint8_t step_counter = 0;
uint8_t dummy_type = DUMMY_ALTERNATING; // 1, 2 or 3
unsigned long timer;


// ----------------------------------------------------------------------------------------------
void trigger_leds();
void follow_line(float steering_factor=0.8);
void rotate_180();
bool reverse_to_line();
bool exit_box();
bool robot_running();
bool exit_rbox();
bool exit_bbox();

// ----------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  
  // Pin modes
  pinMode(SWITCH_PIN, INPUT);
  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(A_LED_PIN, OUTPUT);

  // Setup components and systems
  ls_b.setup(LS_B_PIN);
  ls_fm.setup(LS_FM_PIN);
  ls_fr.setup(LS_FR_PIN);
  ls_fl.setup(LS_FL_PIN);
  ir_r.setup(IR_PIN);
  pt_m.setup(PT_M_PIN);
  acc.setup(ACC_PIN);
  us.setup(US_ECHO_PIN, US_TRIG_PIN);
  claw.setup(CLAW_PIN);
  claw_raiser.setup(CLAW_RAISER_PIN);
  sensor_system.setup();
  sensor_system.add_ls(&ls_b, &ls_fm, &ls_fl, &ls_fr);

  dd_system.setup(&ir_r, &pt_m, nullptr, nullptr);
  drive_system.setup(&AFMS, DC_M_L_PIN, DC_M_R_PIN);
  
  AFMS.begin();

  // Setup LED indicators
  pinMode(A_LED_PIN, OUTPUT);
  digitalWrite(A_LED_PIN, HIGH);
  digitalWrite(G_LED_PIN, 1);
  digitalWrite(R_LED_PIN, 1);

  Serial.println("HELOOOOO");
  // Test suite
  //drive_system.test();
  //sensor_system.test();
  //dd_system.test();
  //us.read();
  //us.test();
  acc.test();
}


void loop() {
//  acc.update_state();
//    Serial.println(acc.get_state());
//    Serial.println(acc.get_ay());
//    
//    delay(500);
//    return;
//  bool ls_readings[4];
//  sensor_system.refresh();
//  sensor_system.get_line_sensors(ls_readings);
    //follow_line();

  // Check if running
  if (!robot_running()) return;

  //Serial.println("HI");
  sensor_system.refresh();
  update_location();
  trigger_leds();

  // Execute tasks
  if (task_counter == 0) {
    Serial.println("Exiting not");
    if (exit_box()) {
      Serial.println("Exiting");
      quad = 0;
      dummy_collected = false;
      dummy_detected = false;
      location_changed = false;
      location = 0;
      tilt = 0;
      drive_system.brake();
      step_counter = 0;
      task_counter++;
      timer = millis();
    }


  } else if (task_counter == 1) {
    Serial.println(millis() - timer);
    if (location == 1) {
      claw_raiser.down();
    } else {
      claw_raiser.up();
    }
    
    if (location == 1 && us.read() < 20) {
      // If dummy detected.
      drive_system.brake();
      step_counter = 0;
      task_counter = 2;
      timer = millis();
    } else if (location == 1 && millis() - timer > 5000 && tilt == 0) {
      // If no dummy detected and reached collection half.
      
      drive_system.brake();
      step_counter = 0;
      task_counter = 3;
      timer = millis();
    }
    follow_line(0.9);


  } else if (task_counter == 2) {
    Serial.println("Seraching for dummy"); 
    // Search for dummy
    locate_dummy();
    // PTmARK
    if (true) {
    //if (pt_m.read() > 200) {
      // Dummy detected when photo transistor registers an analog reading above 200.
      drive_system.brake();
      step_counter = 0;
      task_counter = 3;
      timer = millis();
    }
    
  } else if (task_counter == 3) {
    // Move forward in straight line until dummy reached.
    return;
    if (step_counter == 0) {
      if (us.read() < 20) {
        drive_system.brake();
        step_counter++;
      } else {
        drive_system.forward(DEF_SPEED);
      }
    } else if (step_counter == 1) {
      // Ensure dummy is in line of sight
      drive_system.rotate(80, true);
      timer = millis();
      while (millis() - timer < 2000) {
        if (pt_m.read() > 200) {
          drive_system.brake();
          step_counter = 0;
          task_counter = 4;
          timer = millis();
          return;  
        }
      }
      
      drive_system.rotate(80, false);
      timer = millis();
      while (millis() - timer < 4000) {
        if (pt_m.read() > 200) {
          drive_system.brake();
          step_counter = 0;
          task_counter = 4;
          timer = millis();
          return;  
        }
      }
      
    }

   
  } else if (task_counter == 4) {
    // Identify dummy
    if (step_counter == 0) {
      dummy_type = dd_system.identify_dummy();
      step_counter = 0;
      task_counter = 5;
      timer = millis();
    }

    
  } else if (task_counter == 5) {
    // Collect dummy
    if (step_counter == 0) {
      // Close claw
      claw_raiser.down();
      delay(200);
      claw.close();
      delay(200);
      claw_raiser.up();
      step_counter = 2;
    } else if (step_counter == 2) {
      dummy_collected = true;
      if (dummy_type == DUMMY_MODULATED) {
        digitalWrite(R_LED_PIN, 1);
        digitalWrite(G_LED_PIN, 1);
      } else if (dummy_type == DUMMY_UNMODULATED) {
        digitalWrite(R_LED_PIN, 1);
      } else if (dummy_type == DUMMY_ALTERNATING) {
        digitalWrite(G_LED_PIN, 1);
      }
      step_counter = 0;
      task_counter = 6;
      timer = millis();
    }

    
  } else if (task_counter == 6) {
    // Head to drop off point
    // MARK
    if (step_counter == 0) {
      if (reverse_to_line()) step_counter++;
    } else if (step_counter == 1) {
      Serial.println("Line reached");
      if (dummy_type == DUMMY_MODULATED) {
        if (quad == 1) {
          if (realign(true)) step_counter++;
        } else if (quad == 2) {
          if (realign(false)) step_counter++;
        } else {
          // Robot already aligned.
          step_counter++;
        }
      } else {
        if (quad == 1) {
          if (realign(false)) step_counter++;
        } else if (quad == 2) {
          if (realign(true)) step_counter++;
        } else {
          // Rotate 180;
          drive_system.rotate(DEF_SPEED, true);
          delay(300); // To prevent robot from early triggering due to line sensors at initial position;
          if (realign(true)) step_counter++;
        }
      }
    } else if (step_counter == 2) {
      Serial.println("Aligned");
      if (junction_reached()) step_counter++;
      follow_line();
    } else if (step_counter == 3) {
      Serial.println("Heading to box");
      if (dummy_type == DUMMY_MODULATED) {
        if (enter_wbox()) {
          drive_system.brake();
          step_counter = 0;
          task_counter = 9;
          timer = millis();
        }
      } else if (dummy_type == DUMMY_UNMODULATED) {
        if (enter_bbox()) {
          drive_system.brake();
          step_counter = 0;
          task_counter = 10;
          timer = millis();
        }
      } else {
        if (enter_rbox()) {
          drive_system.brake();
          step_counter = 0;
          task_counter = 10;
          timer = millis();
        }
      }
    }

    
  } else if (task_counter == 7) {
    // Drop dummy in box
    // Open claw
    claw_raiser.down();
    delay(200);
    claw.open();
    delay(200);
    claw_raiser.up();
    n_dummy_collected++;
    dummy_collected = false;
    task_counter = 9;

    
  } else if (task_counter == 9) {
    //Exit white box
    Serial.println("FUCK");
    if (step_counter == 0) {
      if (exit_wbox()) {
          step_counter++;
          timer = millis();
      }
    } else {
      
    }
    if (n_dummy_collected == 3) {
      // Return to box;
      if (step_counter == 1) {
        follow_line();
        if (millis() - timer > 3000) {
          step_counter++;
          timer = millis();
        }
      } else if (step_counter == 2) {
        follow_line();
        if (junction_reached()) {
          delay(1000);
          bool ls_readings[4];
          while(true) {
            sensor_system.refresh();
            sensor_system.get_line_sensors(ls_readings);
            follow_line();
            if (!ls_readings[1] && !ls_readings[2] && !ls_readings[3] && !ls_readings[0]) {
              // Reached box
              Serial.println("Completed");
              drive_system.brake();
              task_counter = 100;
              is_running = false;
              return;
            }
          }
        }
      }
    } else {
      if (step_counter == 1) {
        if (millis() - timer > 3000) {
          // Stop before slope
          drive_system.brake();
          step_counter++;
          timer = millis();
        }
        follow_line();
      } else if (step_counter == 2) {
        // Rotate 180
        rotate_180();
        drive_system.brake();
        step_counter = 0;
        task_counter = 2;
        timer = millis();
      }
    }
  } else if (task_counter == 10) {
    if (step_counter == 0) {
      if (dummy_type == DUMMY_UNMODULATED) {
        if (exit_rbox(n_dummy_collected != 3)) {
          step_counter++;
        }
      } else if (dummy_type == DUMMY_ALTERNATING) {
        if (exit_bbox(n_dummy_collected != 3)) {
          step_counter++;
        }
      }
       
    } else if (step_counter == 1) {
      if (n_dummy_collected == 3) {
        bool ls_readings[4];

        timer = millis();
        while (millis() - timer < 3000) {
          sensor_system.refresh();
          sensor_system.get_line_sensors(ls_readings);
          follow_line();
        }
        
        while(true) {
          sensor_system.refresh();
          sensor_system.get_line_sensors(ls_readings);
          follow_line();
          if (ls_readings[1] && ls_readings[2] && ls_readings[3]) {
            drive_system.forward(DEF_SPEED);
            // Reached box
            delay(2000);
            Serial.println("Completed");
            drive_system.brake();
            task_counter = 100;
            is_running = false;
            return;
          }
        }

      } else {
        drive_system.brake();
        step_counter = 0;
        task_counter = 1; // Search for other dummies
        timer = millis();
      }
    }
  } 
}


void climb_ramp() {
  while(true) {
    sensor_system.refresh();
    drive_system.forward(255);
    if (acc.get_state() == ELEVATED_STATE) {
      drive_system.brake();
      delay(500);
      drive_system.backward(DEF_SPEED);
      while(true) {
        bool ls_readings[4];
        sensor_system.refresh();
        sensor_system.get_line_sensors(ls_readings);
        if (ls_readings[1]) {
          drive_system.brake();
          delay(500);
          return;
        }
      }
    }
  }
}


void trigger_leds() {
  if (!drive_system.get_brake() && is_running) {
    pinMode(A_LED_PIN, INPUT);
  } else {
    pinMode(A_LED_PIN, OUTPUT);
    digitalWrite(A_LED_PIN, HIGH);
  }

  if (!dummy_collected) {
    digitalWrite(G_LED_PIN, LOW);
    digitalWrite(R_LED_PIN, LOW);
  }
}


void update_location() {
  acc.update_state();
  Serial.println("----");
  Serial.println(acc.get_ay());
  Serial.println(acc.get_state());
  Serial.println(location);
  Serial.println("----");
  if (!location_changed && acc.get_state() == DESCEND_STATE) {
    location++;
    location %= 2;
    location_changed = true;
  } else if (location_changed && acc.get_state() == NORMAL_STATE){
    location_changed = false;
  }
}


bool exit_box() {
  bool ls_readings[4];
  sensor_system.get_line_sensors(ls_readings);   
  if (step_counter == 0) {
    drive_system.steer(DEF_SPEED, false);
    if (ls_readings[0]) {
        drive_system.brake();
        step_counter++;
    }
  } else if (step_counter == 1) {
    drive_system.rotate(150, false);
    if (ls_readings[1]) {
      return true;
    }
  }
  return false;
}


void follow_line(float steering_factor) {
  bool ls_readings[4];
  sensor_system.get_line_sensors(ls_readings);  
  Serial.println("Follow line");
  if (tilt == 0) {
    drive_system.forward(DEF_SPEED);
    if (ls_readings[2] && !ls_readings[1] && !ls_readings[3]) {
      drive_system.steer(DEF_SPEED, true);
      tilt = 1;
    } else if (ls_readings[3] && !ls_readings[1] && !ls_readings[2]) {
      drive_system.steer(DEF_SPEED, false);
      tilt = -1;
    }
  } else {
    if (ls_readings[0]) {
      if (tilt == 1) {
        drive_system.steer(DEF_SPEED, true, steering_factor);
      } else {
        drive_system.steer(DEF_SPEED, false, steering_factor);
      }
      
    } else if (!ls_readings[0]) {
      if (tilt == 1) {
        drive_system.steer(DEF_SPEED, true, steering_factor - 0.1);
      } else {
        drive_system.steer(DEF_SPEED, false, steering_factor - 0.1);
      }
    }
    
    if (ls_readings[1] && !ls_readings[2] && !ls_readings[3]) {
      //drive_system.brake();
      tilt = 0;
    }
  }
}

bool realign(bool cw) {
  bool ls_readings[4];
  sensor_system.refresh();
  sensor_system.get_line_sensors(ls_readings);

  drive_system.rotate(DEF_SPEED, cw);
  if (ls_readings[1]) {
    drive_system.brake();
    return true;
  }
  return false;
}


bool reverse_to_line() {
  bool ls_readings[4];
  sensor_system.refresh();
  sensor_system.get_line_sensors(ls_readings);

  drive_system.backward(DEF_SPEED);
  if (ls_readings[0]) {
    drive_system.brake();
    return true;
  }
  return false;
}

bool robot_running() {
  if (!is_running) {
    if (digitalRead(SWITCH_PIN)) {
      Serial.println("Running.");
      delay(500);
      is_running = true;
      claw_raiser.up();
      timer = millis();
      return true;
    }
    return false;
  } else {
    if (digitalRead(SWITCH_PIN)) {
      Serial.println("Stopped.");
      delay(500);
      drive_system.brake();
      is_running = false;
      return false;
    }
    return true;
  }
}


void locate_dummy() {
  bool ls_readings[4];
  sensor_system.refresh();
  sensor_system.get_line_sensors(ls_readings);

  if (step_counter == 0) {
    // Check if dummy in front.
    drive_system.brake();
    step_counter++;
  } else if (step_counter == 1) {
    drive_system.rotate(100, false);
    step_counter++;
  } else if (step_counter == 2) {
    quad = 1;
    if (ls_readings[2]) step_counter++;
  } else if (step_counter == 3) {
    if (ls_readings[1]) step_counter++;
  } else if (step_counter == 4) {
    // Robot facing right quadrant
    quad = 2;
    if (ls_readings[3]) step_counter++;
  } else if (step_counter == 5) {
    if (ls_readings[1]) {
      // Full rotation reached, re-rotate
      step_counter == 1;
    }
  }
}

bool junction_reached() {
  bool ls_readings[4];
  sensor_system.refresh();
  sensor_system.get_line_sensors(ls_readings);

  if (ls_readings[1] && ls_readings[2] && ls_readings[3]) {
    drive_system.brake();
    return true;
  }
  return false;
}

bool enter_wbox() {
  bool ls_readings[4];
  sensor_system.refresh();
  sensor_system.get_line_sensors(ls_readings);

  if (ls_readings[0]) {
    return true;
  }
  return false;
}


bool enter_rbox() {
  bool ls_readings[4];
  sensor_system.refresh();
  drive_system.rotate(DEF_SPEED, true);
  delay(300);
  while(true) {
    sensor_system.refresh();
    sensor_system.get_line_sensors(ls_readings);
    if (ls_readings[2] && ls_readings[3]) {
      drive_system.forward(DEF_SPEED);
      break;
    }
  }
  
  while(true) {
    sensor_system.refresh();
    sensor_system.get_line_sensors(ls_readings);
    if (ls_readings[0]) {
      drive_system.brake();
      break;
    }
  }
  
  if (ls_readings[0]) {
    return true;
  }
  return false;
}


bool enter_bbox() {
  bool ls_readings[4];
  sensor_system.refresh();
  drive_system.rotate(DEF_SPEED, true);
  delay(300);
  while(true) {
    sensor_system.refresh();
    sensor_system.get_line_sensors(ls_readings);
    if (ls_readings[2] && ls_readings[3]) {
      drive_system.forward(DEF_SPEED);
      break;
    }
  }
  while(true) {
    sensor_system.refresh();
    sensor_system.get_line_sensors(ls_readings);
    if (ls_readings[0]) {
      drive_system.brake();
      break;
    }
  }
  
  if (ls_readings[0]) {
    return true;
  }
  return false;
}

bool exit_wbox() {
  bool ls_readings[4];
  sensor_system.refresh();
  sensor_system.get_line_sensors(ls_readings);
  drive_system.backward(DEF_SPEED);
  if (ls_readings[1] && ls_readings[2] && ls_readings[3]) {
      // Rotatate cw until front sensor hits white line
//      rotate_180();
//      drive_system.brake();
//      return true;
      Serial.println("ROTATE");
      drive_system.rotate(DEF_SPEED, true);
      delay(3000);
      sensor_system.refresh();
      while (true) {
        //Need to refresh twice for no reason
        sensor_system.refresh();
        sensor_system.get_line_sensors(ls_readings);
        sensor_system.refresh();
        sensor_system.get_line_sensors(ls_readings);                   
       
        if (ls_readings[1]) {
          Serial.println("DONE");
          drive_system.brake();
          return true;
        }
      }
  }
  return false;
}

bool exit_rbox(bool repeat) {
  bool ls_readings[4];
  sensor_system.refresh();
  sensor_system.get_line_sensors(ls_readings);
  // Move back
  drive_system.backward(DEF_SPEED);
  delay(400); 
  // Rotate 90 cw
  drive_system.rotate(DEF_SPEED, repeat);
  delay(800);
  while (true) {
    if (ls_fm.read()) {
      Serial.println("break");
      break;
    }
  }

  bool on_line = false;
  drive_system.forward(DEF_SPEED);
  timer = millis();
  while (millis() - timer < 2000) {
    if (ls_b.read()) {
      drive_system.brake();
      on_line = true;
      break;
    }
  }

  if (!on_line) {
    drive_system.backward(DEF_SPEED);
    timer = millis();
    while (millis() - timer < 4000) {
      if (ls_b.read()) {
        drive_system.brake();
        on_line = true;
        break;
      } 
    }
  }

  // Realign to line by looking at front line sensor
  drive_system.rotate(DEF_SPEED, !repeat);
  while(true) {
    if (ls_fm.read()) {
      drive_system.brake();
      on_line = true;
      return true;
    }
  } 
}


bool exit_bbox(bool repeat) {
  bool ls_readings[4];
  sensor_system.refresh();
  sensor_system.get_line_sensors(ls_readings);
  // Move back
  drive_system.backward(DEF_SPEED);
  delay(400); 
  // Rotate 90 acw
  drive_system.rotate(DEF_SPEED, !repeat);
  delay(600);
  while (true) {
    if (ls_fm.read()) {
      Serial.println("break");
      break;
    }
  }

  bool on_line = false;
  drive_system.forward(DEF_SPEED);
  timer = millis();
  while (millis() - timer < 2000) {
    if (ls_b.read()) {
      drive_system.brake();
      on_line = true;
      break;
    }
  }

  if (!on_line) {
    drive_system.backward(DEF_SPEED);
    timer = millis();
    while (millis() - timer < 4000) {
      if (ls_b.read()) {
        drive_system.brake();
        on_line = true;
        break;
      } 
    }
  }

  // Realign to line by looking at front line sensor
  drive_system.rotate(DEF_SPEED, repeat);
  while(true) {
    if (ls_fm.read()) {
      drive_system.brake();
      on_line = true;
      return true;
    }
  } 
}


void rotate_180() {
  // Rotatate cw until front sensor hits white line
  drive_system.rotate(DEF_SPEED, true);
  delay(3000);
  bool ls_readings[4];
  while (true) {
    if (ls_fm.read()) {
      Serial.println("break");
      break;
    }
  }

  bool on_line = false;
  drive_system.forward(DEF_SPEED);
  timer = millis();
  while (millis() - timer < 2000) {
    if (ls_b.read()) {
      drive_system.brake();
      on_line = true;
      break;
    }
  }

  if (!on_line) {
    drive_system.backward(DEF_SPEED);
    timer = millis();
    while (millis() - timer < 4000) {
      if (ls_b.read()) {
        drive_system.brake();
        on_line = true;
        break;
      } 
    }
  }

  // Realign to line by looking at front line sensor
  drive_system.rotate(DEF_SPEED, false);
  while(true) {
    if (ls_fm.read()) {
      drive_system.brake();
      on_line = true;
      return;
    }
  }
}
