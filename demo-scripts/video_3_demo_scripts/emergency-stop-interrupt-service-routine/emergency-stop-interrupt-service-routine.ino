/* 
 * Example program demonstrating how interrupt service routine (ISR) can be triggered when emergency stop button is pressed
 * ISR can trigger data logging of event and/or be used to enter the robot into a standby mode while waiting for a safe restart signal
 * 
 * This demo program is primitive in that the ISR only reports the estop current state, it does not log data or wait for a safe restart signal before driving the actuators again.
 * For actual deployment on robotic systems the robot actuators should only be enabled again after the operator provides a direct command (safe restart)
 * 
 * Runs on Arduino Nano with MD10C motor controllers
 * 
 * Copyright (c) 2022 Will Donaldson
 * MIT License (MIT)
 * www.willdonaldson.io
 */
 
#include "motor_driver.h"

#define DirPin1 4
#define DirPin2 11
#define DirPin3 7

#define PWMPin1 5
#define PWMPin2 10
#define PWMPin3 6

#define NCpin 2   // normally closed e-stop pin
#define NOpin 3   // normally open e-stop pin

volatile bool ISR_state_change;

motorDriver M1(DirPin1, PWMPin1);
motorDriver M2(DirPin2, PWMPin2);
motorDriver M3(DirPin3, PWMPin3);

unsigned long debounceDelay = 50; 
unsigned long lastDebounceTime = 0;

#define PI 3.14159265
float R = 0.2776; // robot wheel-base radius
int scaling_factor = 255;  // pseudo scaling factor, in open loop we can't accurately send RPM commands so instead this scales the PWM signal to a reasonable value in the range of [0, 255]

void setup() {
  Serial.begin(115200);
  
  pinMode(NCpin, INPUT);
  pinMode(NOpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(NCpin), shared_estop_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(NOpin), shared_estop_ISR, CHANGE);
  
  check_estop_state(); // manually call the ISR once at setup() to get the initial state of the e-stop button
}

void loop() {
  if(ISR_state_change == true){
    check_estop_state();
    ISR_state_change = false; 
  }
  delay(50);
}

void shared_estop_ISR(){  
  // both hardware pins trigger the same interrupt
  // it may seem redundant to tie both hardware pins to the same ISR, but this is to handle cases in which one of the hardware pin wirings fail
  if ((millis() - lastDebounceTime) > debounceDelay) {
    ISR_state_change = true;
    lastDebounceTime = millis();
  }
  return;
}

void check_estop_state(){
  // emergency-stop status is deduced based on HIGH/LOW state of pins
  // if there is a wiring failure the third conditional will trigger
  int NC_state = digitalRead(NCpin);
  int NO_state = digitalRead(NOpin);
  if(NC_state == 0 && NO_state == 1){
    // estop is engaged; robot cannot move
    calc_speed(0, 0, 0);            // send a software shutdown to actuators, in addition to a hardware (e-stop) shutdown 
    Serial.println("Emergency stop is engaged");
  }
  else if(NC_state == 1 && NO_state == 0){
    // estop is disengaged; robot is free to roam 
    calc_speed(0, -0.5, 0);         // drive robot 
    Serial.println("Emergency stop is released");
  }
  else if((NC_state == 0 && NO_state == 0) || (NC_state == 1 && NO_state == 1)){
    // this is an error state, it should not be possible; wiring on estop may be damaged
    calc_speed(0, 0, 0);            // send a software shutdown to actuators
    Serial.println("Error state. This should not be possible. Check if wiring is correct.");
  }
  ISR_state_change = false;         // reset for next event
}

void calc_speed(float x_dot, float y_dot, float theta_dot){
  // calculates motor speeds for 3 wheel omni drive robot
  // see derivation of equations here: https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/
  float PWM1 = -R*theta_dot + x_dot ;
  float PWM2 = -R*theta_dot - 0.5*x_dot - sin(PI/3.0)*y_dot;
  float PWM3 = -R*theta_dot - 0.5*x_dot + sin(PI/3.0)*y_dot;
  
  int Dir1 = (PWM1 > 0) - (PWM1 < 0);   // returns -1 or 1
  int Dir2 = (PWM2 > 0) - (PWM2 < 0);
  int Dir3 = (PWM3 > 0) - (PWM3 < 0);

  PWM1 = (int)abs(PWM1*scaling_factor); // scale in PWM range of [0, 255]
  PWM2 = (int)abs(PWM2*scaling_factor);
  PWM3 = (int)abs(PWM3*scaling_factor);
  
  M1.spin(Dir1, PWM1);
  M2.spin(Dir2, PWM2);
  M3.spin(Dir3, PWM3);
}
