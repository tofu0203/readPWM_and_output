#ifndef BrushlessMotor_h
#define BrushlessMotor_h
#include <Servo.h>

class BrushlessMotor {
  public:
    BrushlessMotor(int pin);
    
    void ReadPWM();
    int GetMotorPin();
    unsigned int GetPwmInput();
    
  private:
    int _MotorPin;
    volatile  unsigned long _Up = 0;
    volatile  unsigned long _Down = 0;
    volatile unsigned int _pwm_input = 0;
};

#endif
