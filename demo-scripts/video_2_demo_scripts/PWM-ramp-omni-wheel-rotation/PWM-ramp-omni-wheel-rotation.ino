/* 
 * Example program of driving 3 omni wheels in a PWM ramp to rotate holonomic robot on the spot in an open loop
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

int stallPWM = 30;
int stepDelay = 7;

void setup() {
}

void loop() {
  delay(1000);
  for(int i = stallPWM; i <= 255; i++){
    M1.spin(1, i);
    M2.spin(1, i);
    M3.spin(1, i);
    delay(stepDelay);
  }
  delay(1000);
  for(int i = 255; i >= stallPWM; i--){
    M1.spin(1, i);
    M2.spin(1, i);
    M3.spin(1, i);
    delay(stepDelay);
  }
}
