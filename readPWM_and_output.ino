#include "BrushlessMotor.h"
BrushlessMotor input_motor = BrushlessMotor(35);
void setup()
{
    Serial.begin(115200);
    attachInterrupt(input_motor.GetMotorPin(), SUB_input_motor, CHANGE);
}
void loop()
{
    Serial.println(input_motor.GetPwmInput());
    delay(100);
}
void SUB_input_motor()
{
    input_motor.ReadPWM();
}