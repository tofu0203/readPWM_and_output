#define USE_USBCON //dueÅÊM·éÆ«ÉKv
#include "Servo.h"
#include "BrushlessMotor.h"

// -----adafruit_pwm_servo_driver--------
#include <Wire.h>
#include "my_Adafruit_PWMServoDriver.h"
#define SERVO_FREQ 200
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//------ros-------
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
ros::NodeHandle nh;

std_msgs::Int16MultiArray array_msg;
ros::Publisher arduino_data("array_pub", &array_msg);
volatile long last_update_time = 0;
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
const int m1_output_shieldpin = 2;
const int m2_output_shieldpin = 3;
const int m3_output_shieldpin = 4;
const int m4_output_shieldpin = 5;
Servo m1;
Servo m2;
Servo m3;
Servo m4;

int brushless1_command;
int brushless2_command;
int brushless3_command;
int brushless4_command;

//lateral rotor
//lateral brushless motor pin
const int lateral1_brushless_shiledpin = 4;
const int lateral2_brushless_shiledpin = 5;
//lateral servo motor pin
const int lateral1_servo1_shieldpin = 6;
const int lateral1_servo2_shieldpin = 7;
const int lateral1_servo3_shieldpin = 8;
const int lateral2_servo1_shieldpin = 9;
const int lateral2_servo2_shieldpin = 10;
const int lateral2_servo3_shieldpin = 11;

float lateral1_force = 0.0;
float lateral2_force = 0.0;

float roll_d_gain = 18.0; //18.0;
float roll_velocity = 0.0;

//servo_command=lateral1_thrust_to_servo_command(thrust)--------------------------------------
const float lateral1_param1[4] = {1431.1181858358468, -11.132199571700767, -0.35918663681241014, -0.016709300979755426};
const float lateral1_param2[4] = {1424.1623587070135, -38.13521287160256, -17.562850390397216, 18.769075024676567};
const float lateral1_param3[4] = {1397.8641637953551, -11.269289655814646, 0.3962092075307737, -0.016555344338769373};
int lateral1_thrust_to_servo_command(float thrust)
{
    int command;
    //limitter
    if (thrust > 15.0)
    {
        thrust = 15.0;
    }
    else if (thrust < -15.0)
    {
        thrust = -15.0;
    }
    float x1 = thrust;
    float x2 = thrust * x1;
    float x3 = thrust * x2;
    if (thrust <= -0.45126)
    {
        command = int(lateral1_param1[0] + lateral1_param1[1] * x1 + lateral1_param1[2] * x2 + lateral1_param1[3] * x3);
    }
    else if (-0.45126 < thrust && thrust <= 1.0791)
    {
        command = int(lateral1_param2[0] + lateral1_param2[1] * x1 + lateral1_param2[2] * x2 + lateral1_param2[3] * x3);
    }
    else if (1.0791 < thrust)
    {
        command = int(lateral1_param3[0] + lateral1_param3[1] * x1 + lateral1_param3[2] * x2 + lateral1_param3[3] * x3);
    }
    return command;
}
const float lateral2_param1[4] = {1332.647258536379, -11.011548736056284, -0.5290815914172526, -0.0191764264997118};
const float lateral2_param2[4] = {1330.5815949288717, -35.44922932778718, -60.37887213536305, 76.42089875279635};
const float lateral2_param3[4] = {1309.757174467697, -10.841931780203595, 0.4358451117468576, -0.0140146922403801};
int lateral2_thrust_to_servo_command(float thrust)
{
    int command;
    //limitter
    if (thrust > 15.0)
    {
        thrust = 15.0;
    }
    else if (thrust < -15.0)
    {
        thrust = -15.0;
    }
    float x1 = thrust;
    float x2 = thrust * x1;
    float x3 = thrust * x2;
    if (thrust <= -0.15696)
    {
        command = int(lateral2_param1[0] + lateral2_param1[1] * x1 + lateral2_param1[2] * x2 + lateral2_param1[3] * x3);
    }
    else if (-0.15696 < thrust && thrust <= 0.6867)
    {
        command = int(lateral2_param2[0] + lateral2_param2[1] * x1 + lateral2_param2[2] * x2 + lateral2_param2[3] * x3);
    }
    else if (0.6867 < thrust)
    {
        command = int(lateral2_param3[0] + lateral2_param3[1] * x1 + lateral2_param3[2] * x2 + lateral2_param3[3] * x3);
    }
    return command;
}
//------------------------------------------------------
// ブラシレスモーターの推力からコマンドに変換する関数
float brushless_force_to_command(float force)
{
    float N_to_gf = 101.971;
    //横軸コマンド・縦軸推力の時、傾きが6.4198だった
    float force_to_command_gradient = 1.0 / 6.4189;
    return force * 101.971 * force_to_command_gradient;
}

int mode = 0; //mode==0のとき水平方向ロータなし、mode==1のとき水平方向ロータありyaw制御、mode==2のとき水平方向ロータありx_yaw制御
enum flight_mode
{
    without_lateral,
    with_lateral_yaw,
    with_lateral_x_yaw
};

int input_pwm[4];
int transform_pwm[4];
float calc_input_pwm[4];
bool output_motor()
{
    if (millis() - last_update_time > 500)
    {
        brushless1_command = 940;
        brushless2_command = 940;
        brushless3_command = 940;
        brushless4_command = 940;
        m1.writeMicroseconds(brushless1_command);
        m2.writeMicroseconds(brushless2_command);
        m3.writeMicroseconds(brushless3_command);
        m4.writeMicroseconds(brushless4_command);
        return false;
    }
    //DJIよみとった値
    // input_pwm[0] = input_motor1.GetPwmInput();
    // input_pwm[1] = input_motor2.GetPwmInput();
    // input_pwm[2] = input_motor3.GetPwmInput();
    // input_pwm[3] = input_motor4.GetPwmInput();
    input_pwm[0] = (int)((float)input_motor1.GetPwmInput() + roll_d_gain * roll_velocity);
    input_pwm[1] = (int)((float)input_motor2.GetPwmInput() - roll_d_gain * roll_velocity);
    input_pwm[2] = (int)((float)input_motor3.GetPwmInput() - roll_d_gain * roll_velocity);
    input_pwm[3] = (int)((float)input_motor4.GetPwmInput() + roll_d_gain * roll_velocity);
    //yaw操作量を打ち消したもの
    calc_input_pwm[0] = (float)input_motor1.GetPwmInput() + roll_d_gain * roll_velocity;
    calc_input_pwm[1] = (float)input_motor2.GetPwmInput() + roll_d_gain * roll_velocity;
    calc_input_pwm[2] = (float)input_motor3.GetPwmInput() + roll_d_gain * roll_velocity;
    calc_input_pwm[3] = (float)input_motor4.GetPwmInput() + roll_d_gain * roll_velocity;
    transform_pwm[0] = (int)(0.25 * (3.0 * calc_input_pwm[0] + calc_input_pwm[1] - calc_input_pwm[2] + calc_input_pwm[3]));
    transform_pwm[1] = (int)(0.25 * (calc_input_pwm[0] + 3.0 * calc_input_pwm[1] + calc_input_pwm[2] - calc_input_pwm[3]));
    transform_pwm[2] = (int)(0.25 * (-calc_input_pwm[0] + calc_input_pwm[1] + 3.0 * calc_input_pwm[2] + calc_input_pwm[3]));
    transform_pwm[3] = (int)(0.25 * (calc_input_pwm[0] - calc_input_pwm[1] + calc_input_pwm[2] + 3.0 * calc_input_pwm[3]));
    if (mode == without_lateral) //水平方向ロータを使わないとき
    {
        //DJIよみとった値をそのまま出力
        brushless1_command = input_pwm[0];
        brushless2_command = input_pwm[1];
        brushless3_command = input_pwm[2];
        brushless4_command = input_pwm[3];
    }
    else if ((mode == with_lateral_yaw) || (mode == with_lateral_x_yaw)) //æ°´å¹³æ¹åã­ã¼ã¿ãä½¿ãã¨ã
    {
    //鉛直方向ロータ：yaw操作量をなくしたものを出力
        brushless1_command = transform_pwm[0];
        brushless2_command = transform_pwm[1];
        brushless3_command = transform_pwm[2];
        brushless4_command = transform_pwm[3];

        //鉛直方向ロータが回り始めた時に水平方向ロータを回す
        if ((input_pwm[0] >= 1150.0) && (input_pwm[1] >= 1150.0))
        {
            //水平方向ロータ１コマンド値計算
            int lateral1_servo_command = lateral1_thrust_to_servo_command(lateral1_force);
            int lateral1_servo1_command = 3000 - lateral1_servo_command;
            int lateral1_servo2_command = lateral1_servo_command;
            int lateral1_servo3_command = lateral1_servo_command;

                //水平方向ロータ２コマンド値計算
            int lateral2_servo_command = lateral2_thrust_to_servo_command(lateral2_force);
            int lateral2_servo1_command = 3000 - lateral2_servo_command;
            int lateral2_servo2_command = lateral2_servo_command;
            int lateral2_servo3_command = lateral2_servo_command;

            // //============================重心が水平方向ロータから50mm高いためフィードフォワード制御を入れる===============================================
            // float ff_gain = 0.5; //フィードフォワードゲイン
            // //質量中心から鉛直ロータまで0.45m,4つのロータで分担,水平成分だけ補正
            // brushless1_command -= brushless_force_to_command(ff_gain * 0.050 / 0.450 / 4.0 * (lateral1_force + lateral2_force));
            // brushless2_command -= brushless_force_to_command(ff_gain * 0.050 / 0.450 / 4.0 * (lateral1_force + lateral2_force));
            // brushless3_command += brushless_force_to_command(ff_gain * 0.050 / 0.450 / 4.0 * (lateral1_force + lateral2_force));
            // brushless4_command += brushless_force_to_command(ff_gain * 0.050 / 0.450 / 4.0 * (lateral1_force + lateral2_force));
            // //=============================================================================================================================================

              //水平方向ロータ出力
            pwm.writeMicroseconds(lateral1_brushless_shiledpin, 1800);
            pwm.writeMicroseconds(lateral2_brushless_shiledpin, 1800);
            pwm.writeMicroseconds(lateral1_servo1_shieldpin, lateral1_servo1_command);
            pwm.writeMicroseconds(lateral1_servo2_shieldpin, lateral1_servo2_command);
            pwm.writeMicroseconds(lateral1_servo3_shieldpin, lateral1_servo3_command);
            pwm.writeMicroseconds(lateral2_servo1_shieldpin, lateral2_servo1_command);
            pwm.writeMicroseconds(lateral2_servo2_shieldpin, lateral2_servo2_command);
            pwm.writeMicroseconds(lateral2_servo3_shieldpin, lateral2_servo3_command);
        }
        else
        {
            //水平方向ロータ１コマンド値計算
            int lateral1_servo_command = lateral1_thrust_to_servo_command(0.0);
            int lateral1_servo1_command = 3000 - lateral1_servo_command;
            int lateral1_servo2_command = lateral1_servo_command;
            int lateral1_servo3_command = lateral1_servo_command;

            //水平方向ロータ２コマンド値計算
            int lateral2_servo_command = lateral2_thrust_to_servo_command(0.0);
            int lateral2_servo1_command = 3000 - lateral2_servo_command;
            int lateral2_servo2_command = lateral2_servo_command;
            int lateral2_servo3_command = lateral2_servo_command;

            //水平方向ロータ出力
            pwm.writeMicroseconds(lateral1_brushless_shiledpin, 1000);
            pwm.writeMicroseconds(lateral2_brushless_shiledpin, 1000);
            pwm.writeMicroseconds(lateral1_servo1_shieldpin, lateral1_servo1_command);
            pwm.writeMicroseconds(lateral1_servo2_shieldpin, lateral1_servo2_command);
            pwm.writeMicroseconds(lateral1_servo3_shieldpin, lateral1_servo3_command);
            pwm.writeMicroseconds(lateral2_servo1_shieldpin, lateral2_servo1_command);
            pwm.writeMicroseconds(lateral2_servo2_shieldpin, lateral2_servo2_command);
            pwm.writeMicroseconds(lateral2_servo3_shieldpin, lateral2_servo3_command);
        }
    }
    m2.writeMicroseconds(brushless2_command);
    m1.writeMicroseconds(brushless1_command);
    m3.writeMicroseconds(brushless3_command);
    m4.writeMicroseconds(brushless4_command);
    return true;
}

void arduinoCallback(const std_msgs::Float32MultiArray &command_value)
{
    mode = (int)command_value.data[0];
    float u_x = command_value.data[1];
    float u_yaw = command_value.data[2];
    if (mode == flight_mode::without_lateral)
    {
        lateral1_force = 0.0;
        lateral2_force = 0.0;
    }
    else if (mode == flight_mode::with_lateral_yaw)
    {
        lateral1_force = u_yaw;
        lateral2_force = -u_yaw;
    }
    else if (mode == flight_mode::with_lateral_x_yaw)
    {
        lateral1_force = u_x + u_yaw;
        lateral2_force = u_x - u_yaw;
    }
    // nh.logwarn(String(brushless1_command).c_str());
}

void angularVelocityCallback(const geometry_msgs::Vector3Stamped &angular_velocity)
{
    roll_velocity = angular_velocity.vector.x;
    // nh.logwarn(String(roll_d_gain * roll_velocity).c_str());
}
ros::Subscriber<std_msgs::Float32MultiArray> sub1("arduino_control", &arduinoCallback);
ros::Subscriber<geometry_msgs::Vector3Stamped> sub2("/dji_sdk/angular_velocity_fused", &angularVelocityCallback);

void setup()
{
    //-----ros------
    array_msg.data_length = 4;
    array_msg.data = (int16_t *)malloc(sizeof(int16_t) * 4);

    // nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(arduino_data);
    nh.subscribe(sub1);
    nh.subscribe(sub2);

    //   ---PWMshield--------
    pwm.begin();
    pwm.setOscillatorFrequency(25000000);
    pwm.setPWMFreq(SERVO_FREQ);
    Wire.setClock(400000);
    pwm.writeMicroseconds(lateral1_brushless_shiledpin, 1000);
    pwm.writeMicroseconds(lateral2_brushless_shiledpin, 1000);

    // ---servo library-------------
    m1.attach(m1_output_shieldpin);
    m2.attach(m2_output_shieldpin);
    m3.attach(m3_output_shieldpin);
    m4.attach(m4_output_shieldpin);

    //-----割り込み------
    attachInterrupt(input_motor1.GetMotorPin(), SUB_input_motor1, CHANGE);
    attachInterrupt(input_motor2.GetMotorPin(), SUB_input_motor2, CHANGE);
    attachInterrupt(input_motor3.GetMotorPin(), SUB_input_motor3, CHANGE);
    attachInterrupt(input_motor4.GetMotorPin(), SUB_input_motor4, CHANGE);
}
bool success_output = false;
void loop()
{
    if (!nh.connected())
    { //-----------------------------------------------
        brushless1_command = 1000;
        brushless2_command = 1000;
        brushless3_command = 1000;
        brushless4_command = 1000;
        m1.writeMicroseconds(brushless1_command);
        m2.writeMicroseconds(brushless2_command);
        m3.writeMicroseconds(brushless3_command);
        m4.writeMicroseconds(brushless4_command);
        nh.spinOnce();
    }
    else
    {
        array_msg.data[0] = input_motor1.GetPwmInput();
        array_msg.data[1] = input_motor2.GetPwmInput();
        array_msg.data[2] = input_motor3.GetPwmInput();
        array_msg.data[3] = input_motor4.GetPwmInput();
        success_output = output_motor();
        // nh.logwarn(String(array_msg.data[0]).c_str());
        //  nh.logwarn(String(array_msg.data[1]).c_str());
        //   nh.logwarn(String(array_msg.data[2]).c_str());
        //    nh.logwarn(String(array_msg.data[3]).c_str());
        arduino_data.publish(&array_msg);
        nh.spinOnce();
    }
    delayMicroseconds(500);
}

// -----割り込みサブルーチン------
void SUB_input_motor1()
{
    last_update_time = millis();
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