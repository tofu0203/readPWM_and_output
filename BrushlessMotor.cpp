#include "Arduino.h"
#include "BrushlessMotor.h"

BrushlessMotor::BrushlessMotor(int pin)
{
  _MotorPin = pin;
  pinMode(_MotorPin, INPUT_PULLUP);
}

int BrushlessMotor::GetMotorPin(void)
{
  return _MotorPin;
}

unsigned int BrushlessMotor::GetPwmInput(void)
{
  return _pwm_input;
}

// PWMを読み取る。
// 割り込みの関数の中で実行する
void BrushlessMotor::ReadPWM(void)
{
  // ポートにアクセスしHIGHかLOWか読み取る。
  // DegitalRead()と同じだがこちらのほうが速い。
  if (g_APinDescription[_MotorPin].pPort->PIO_PDSR & g_APinDescription[_MotorPin].ulPin)
  {
    _Up = micros();
  }
  else
  {
    _Down = micros();
    _pwm_input = _Down - _Up;
  }
}
