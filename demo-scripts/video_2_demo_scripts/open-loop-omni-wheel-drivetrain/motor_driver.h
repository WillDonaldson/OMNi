#ifndef motordriver_h
#define motordriver_h

#include "Arduino.h"

class motorDriver{
  public:
    motorDriver(int DirPin, int PWMPin);
    void spin(char Dir, unsigned char PWM);
  private:
    int _DirPin;
    int _PWMPin;
};

#endif
