#define USE_USBCON //dueで通信するときに必要
#include <Servo.h>
#include "BrushlessMotor.h"

// -----adafruit_pwm_servo_driver--------
#include <Wire.h>
#include "my_Adafruit_PWMServoDriver.h"
#define SERVO_FREQ 450
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//------ros-------
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
std_msgs::Float32MultiArray array_msg;
ros::Publisher arduino_data("array_pub", &array_msg);

//------input motor setting------
const int m1_input_pin = 28;
const int m2_input_pin = 36;
const int m3_input_pin = 44;
const int m4_input_pin = 52;

BrushlessMotor input_motor1 = BrushlessMotor(m1_input_pin);
BrushlessMotor input_motor2 = BrushlessMotor(m2_input_pin);
BrushlessMotor input_motor3 = BrushlessMotor(m3_input_pin);
BrushlessMotor input_motor4 = BrushlessMotor(m4_input_pin);

//------output motor setting------
const int m1_output_shieldpin = 0;
const int m2_output_shieldpin = 1;
const int m3_output_shieldpin = 2;
const int m4_output_shieldpin = 3;

int brushless1_command;
int brushless2_command;
int brushless3_command;
int brushless4_command;

// //lateral rotor
// Servo lateral1_brushlessmotor;
// Servo lateral2_brushlessmotor;
// //lateral Servo
// Servo lateral1_servo1;
// Servo lateral1_servo2;
// Servo lateral1_servo3;
// Servo lateral2_servo1;
// Servo lateral2_servo2;
// Servo lateral2_servo3;
// int lateral1_servo_command;
// int lateral1_servo1_command;
// int lateral1_servo2_command;
// int lateral1_servo3_command;
// int lateral2_servo_command;
// int lateral2_servo1_command;
// int lateral2_servo2_command;
// int lateral2_servo3_command;

void yawCallback(const std_msgs::Float32MultiArray &command_value)
{
    brushless1_command = input_motor1.GetPwmInput();
    brushless2_command = input_motor2.GetPwmInput();
    brushless3_command = input_motor3.GetPwmInput();
    brushless4_command = input_motor4.GetPwmInput();

    // output motor
    pwm.writeMicroseconds(m1_output_shieldpin, brushless1_command);
    pwm.writeMicroseconds(m2_output_shieldpin, brushless2_command);
    pwm.writeMicroseconds(m3_output_shieldpin, brushless3_command);
    pwm.writeMicroseconds(m4_output_shieldpin, brushless4_command);
    // nh.loginfo(String(brushless1_command).c_str());
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("yaw_command", &yawCallback);

void setup()
{
    //-----ros------
    array_msg.data_length = 8;
    array_msg.data = (float *)malloc(sizeof(float) * 8);

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(arduino_data);

    //   ---PWMshield--------
    pwm.begin();
    pwm.setOscillatorFrequency(25000000);
    pwm.setPWMFreq(SERVO_FREQ);
    pwm.writeMicroseconds(m1_output_shieldpin, 1000);
    pwm.writeMicroseconds(m2_output_shieldpin, 1000);
    pwm.writeMicroseconds(m3_output_shieldpin, 1000);
    pwm.writeMicroseconds(m4_output_shieldpin, 1000);

    //-----割り込み------
    attachInterrupt(input_motor1.GetMotorPin(), SUB_input_motor1, CHANGE);
    attachInterrupt(input_motor2.GetMotorPin(), SUB_input_motor2, CHANGE);
    attachInterrupt(input_motor3.GetMotorPin(), SUB_input_motor3, CHANGE);
    attachInterrupt(input_motor4.GetMotorPin(), SUB_input_motor4, CHANGE);
}

void loop()
{
    if (!nh.connected())
    { //-----------------------------------------------
        brushless1_command = 1000;
        brushless2_command = 1000;
        brushless3_command = 1000;
        brushless4_command = 1000;
        pwm.writeMicroseconds(m1_input_pin, brushless1_command);
        pwm.writeMicroseconds(m2_input_pin, brushless2_command);
        pwm.writeMicroseconds(m3_input_pin, brushless3_command);
        pwm.writeMicroseconds(m4_input_pin, brushless4_command);
    }
    else
    {
        brushless1_command = input_motor1.GetPwmInput();
        brushless2_command = input_motor2.GetPwmInput();
        brushless3_command = input_motor3.GetPwmInput();
        brushless4_command = input_motor4.GetPwmInput();

        // output motor
        pwm.writeMicroseconds(m1_output_shieldpin, brushless1_command);
        pwm.writeMicroseconds(m2_output_shieldpin, brushless2_command);
        pwm.writeMicroseconds(m3_output_shieldpin, brushless3_command);
        pwm.writeMicroseconds(m4_output_shieldpin, brushless4_command);
        //ros output
        array_msg.data[0] = brushless1_command;
        array_msg.data[1] = brushless2_command;
        array_msg.data[2] = brushless3_command;
        array_msg.data[3] = brushless4_command;
        array_msg.data[4] = 0.25 * (3 * brushless1_command + brushless2_command - brushless3_command + brushless4_command);
        array_msg.data[5] = 0.25 * (brushless1_command + 3 * brushless2_command + brushless3_command - brushless4_command);
        array_msg.data[6] = 0.25 * (-brushless1_command + brushless2_command + 3 * brushless3_command + brushless4_command);
        array_msg.data[7] = 0.25 * (brushless1_command - brushless2_command + brushless3_command + 3 * brushless4_command);
        arduino_data.publish(&array_msg);
    }
    nh.spinOnce();
    delayMicroseconds(500);
}

//-----割り込みサブルーチン------
void SUB_input_motor1()
{
    input_motor1.ReadPWM();
}
void SUB_input_motor2()
{
    input_motor2.ReadPWM();
}
void SUB_input_motor3()
{
    input_motor3.ReadPWM();
}
void SUB_input_motor4()
{
    input_motor4.ReadPWM();
}