/* 
 * Example program demonstrating how the emergency stop button can be polled using Timer1
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include "motor_driver.h"

// IMPORTANT NOTE: This program has different pin allocations (pin 10/11 are swapped) than other examples in the repo
// This is to avoid conflicts with the registers of Timer1
// You must rewire the connections as defined in the following pin allocations
#define DirPin1 4
#define DirPin2 10
#define DirPin3 7

#define PWMPin1 5
#define PWMPin2 11
#define PWMPin3 6

#define NCpin 2   // normally closed e-stop pin
#define NOpin 3   // normally open e-stop pin

motorDriver M1(DirPin1, PWMPin1);
motorDriver M2(DirPin2, PWMPin2);
motorDriver M3(DirPin3, PWMPin3);

volatile int estop_status;

const uint16_t TIMER1_PRESCALER = 64;
const uint32_t CPU_FREQUENCY = F_CPU;
const uint32_t INTERRUPT_FREQUENCY = 20; // 20Hz (50ms)

#define PI 3.14159265
float R = 0.2776; // robot wheel-base radius
int scaling_factor = 255;  // pseudo scaling factor, in open loop we can't accurately send RPM commands so instead this scales the PWM signal to a reasonable value in the range of [0, 255]

void setup() {
  Serial.begin(115200);

  pinMode(PWMPin1, OUTPUT);
  pinMode(PWMPin2, OUTPUT);
  pinMode(PWMPin3, OUTPUT);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  pinMode(DirPin3, OUTPUT);
  pinMode(NCpin, INPUT);
  pinMode(NOpin, INPUT);
  
  // Timer1 settings for interrupt
  TCCR1A = 0;                           // Set to normal mode
  TCCR1B = 0;                           // Clear settings
  TCNT1 = 0;                            // Reset timer1 counter
  uint32_t timer1CompareValue = (CPU_FREQUENCY / (TIMER1_PRESCALER * INTERRUPT_FREQUENCY)) - 1;
  OCR1A = timer1CompareValue;           // Calculate and set the compare match register value
  TCCR1B |= (1 << WGM12);               // Enable CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);  // Set prescaler to 64
  TIMSK1 |= (1 << OCIE1A);              // Enable the compare match interrupt
  sei();                                // Enable global interrupts
}

void loop() {
  check_estop_state();
  delay(50);
}

ISR(TIMER1_COMPA_vect) {
  poll_estop();
}

void poll_estop(){
  // emergency-stop status is deduced based on HIGH/LOW state of pins
  // if there is a wiring failure the third conditional will trigger
  int NC_state = digitalRead(NCpin);
  int NO_state = digitalRead(NOpin);
  if(NC_state == 0 && NO_state == 1){
    // estop is engaged; robot cannot move
    estop_status = 0;
    return;
  }
  else if(NC_state == 1 && NO_state == 0){
    // estop is disengaged; robot is free to roam 
    estop_status = 1;
    return;
  }
  else if((NC_state == 0 && NO_state == 0) || (NC_state == 1 && NO_state == 1)){
    // this is an error state, it should not be possible; wiring on estop may be damaged
    estop_status = 2;
    return;
  }
}

void check_estop_state(){
  // emergency-stop status is deduced based on HIGH/LOW state of pins
  // if there is a wiring failure the third conditional will trigger
  int current_estop_status = estop_status;      // convert to non-volatile int for duration on function 
  switch(current_estop_status){
    case 0:
      calc_speed(0, 0, 0);            // send a software shutdown to actuators, in addition to a hardware (e-stop) shutdown 
      Serial.println("Emergency stop is engaged");
      break;
    case 1:
      calc_speed(0, -0.5, 0);         // drive robot 
      Serial.println("Emergency stop is released");
      break;
    case 2:
      calc_speed(0, 0, 0);            // send a software shutdown to actuators
      Serial.println("Error state. This should not be possible. Check if wiring is correct.");
      break;
    default:
      Serial.println("Unexpected error");
      break;
  }
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
