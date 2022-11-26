#include "Arduino.h"
#include "motor_driver.h"

motorDriver::motorDriver(int DirPin, int PWMPin){
  _DirPin = DirPin;
  _PWMPin = PWMPin;
  pinMode(_DirPin, OUTPUT);
  pinMode(_PWMPin, OUTPUT);
}

void motorDriver::spin(char Dir, unsigned char PWM){
  switch (Dir){
    case -1:    // counterclockwise
      digitalWrite(_DirPin, LOW);
      analogWrite(_PWMPin, PWM);
      break;
    case 0:     // stationary
      digitalWrite(_DirPin, LOW);
      digitalWrite(_PWMPin, LOW);
      break;
    case 1:     // clockwise
      digitalWrite(_DirPin, HIGH);
      analogWrite(_PWMPin, PWM);
      break;
  }
}
