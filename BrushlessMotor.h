#ifndef BrushlessMotor_h
#define BrushlessMotor_h
#include <Servo.h>
// Arduino due pinmapping https://www.arduino.cc/en/Hacking/PinMappingSAM3X
class BrushlessMotor
{
public:
  BrushlessMotor(int pin);

  void ReadPWM();
  int GetMotorPin();
  int GetPwmInput();

private:
  int _MotorPin;
  volatile unsigned long _Up = 0;
  volatile unsigned long _Down = 0;
  volatile unsigned int _pwm_input = 0;
};

#endif
