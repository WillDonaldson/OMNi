/* 
 * Example program of driving 3 omni wheels in a holonomic robot configuration in an open loop
 * This program is for demonstration purposes only, since it runs in an open loop without encoder feedback it is not robust
 * 
 * Being open loop, the setpoint RPM calculations in calc_speed() are effectively meaningless. These are instead just values in the range of (0, 255)
 * Note that motor PWM signals don't scale linearly with RPM, as such closed-loop encoder feedback is required for robust control 
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

motorDriver M1(DirPin1, PWMPin1);
motorDriver M2(DirPin2, PWMPin2);
motorDriver M3(DirPin3, PWMPin3);

#define PI 3.14159265
float R = 0.2776; // robot wheel-base radius
int scaling_factor = 255;  // pseudo scaling factor, in open loop we can't accurately send RPM commands so instead this scales the PWM signal to a reasonable value in the range of [0, 255]

void setup() {
  calc_speed(0, -0.5, 0);
}

void loop() {
  
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
