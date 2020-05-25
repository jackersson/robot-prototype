
#include "Arduino.h"
#include "Kinematics.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"

#define COMMAND_RATE 20 //hz

// PID
#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

// Diff Drive
#define MAX_RPM 330              // motor's maximum RPM
#define WHEEL_DIAMETER 0.13      // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.255 // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0     // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define COUNTS_PER_REV 1000      // wheel encoder's no of ticks per rev

// Encoders
// left
#define MOTOR1_ENCODER_A 2
#define MOTOR1_ENCODER_B 15

// right
#define MOTOR2_ENCODER_A 3
#define MOTOR2_ENCODER_B 14

// L298N
#define MOTOR1_PWM 10
#define MOTOR1_IN_A 11
#define MOTOR1_IN_B 12

#define MOTOR2_PWM 9
#define MOTOR2_IN_A 8
#define MOTOR2_IN_B 7

#define PWM_BITS 8 // PWM Resolution of the microcontroller
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV);

Controller motor1_controller(Controller::L298, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::L298, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

void cmd_vel(float linear_x, float angular_z)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = linear_x;
    g_req_angular_vel_z = angular_z;

    g_prev_command_time = millis();
}

void setup()
{
    Serial.begin(9600);

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
}


float to_radians(float angle){
    return angle  * 2 * 3.1415926535897 /360;
}

// the loop function runs over and over again forever
// open serial monitor: https://www.youtube.com/watch?v=Pc7-nkoot48
void loop()
{

    static unsigned long prev_control_time = 0;
    static bool do_once = false;
    static int state = 0;

    // forward
    if (!do_once && (state == 0 || state == 2 || state == 4 || state == 6)){
        cmd_vel(0.5, 0);
        digitalWrite(LED_BUILTIN, HIGH);
        do_once = true;
        Serial.print("Forward");
    }

    if (!do_once && (state == 1 || state == 3 || state == 5 || state == 7)){
        cmd_vel(0, to_radians(30));
        digitalWrite(LED_BUILTIN, HIGH);
        do_once = true;
        Serial.print("Turn");
    }

    //this block drives the robot based on defined rate
    if (do_once && (millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
        Serial.print("moveBase");
        // Serial.print("\t");
    }

    //this block stops the motor when no command is received
    if (do_once && (millis() - g_prev_command_time) >= 3000)
    {
        stopBase();
        state++;
        digitalWrite(LED_BUILTIN, LOW);
        do_once = false;
        Serial.print("StopBase");
        // Serial.print("\t");
    }

    if (state > 7)
        state = 0;

}

void print_goal(){
    Serial.print(g_req_linear_vel_x);
    Serial.print("\t");
    Serial.print(g_req_angular_vel_z);
    Serial.print("\t");
    Serial.println();
}

void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor
    int current_rpm1 = motor1_encoder.getRPM() * -1;
    int current_rpm2 = motor2_encoder.getRPM();
    int current_rpm3 = 0;
    int current_rpm4 = 0;

    // if (current_rpm1 != 0 || current_rpm2 != 0)
    // {
    //     Serial.print(current_rpm1);
    //     Serial.print("\t");
    //     Serial.print(current_rpm2);
    //     Serial.print("\t");
    //     Serial.println();
    // }

    // if (req_rpm.motor1 != 0 || req_rpm.motor2 != 0)
    // {
    //     Serial.print(req_rpm.motor1);
    //     Serial.print("\t");
    //     Serial.print(req_rpm.motor2);
    //     Serial.print("\t");
    //     Serial.println();
    // }

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    float m1_target = motor1_pid.compute(req_rpm.motor1, current_rpm1);
    float m2_target = motor2_pid.compute(req_rpm.motor2, current_rpm2);
    // if (m1_target != 0 || m2_target != 0)
    // {
    //     Serial.print(m1_target);
    //     Serial.print("\t");
    //     Serial.print(m2_target);
    //     Serial.print("\t");
    //     Serial.println();
    // }
    motor1_controller.spin(m1_target);
    motor2_controller.spin(m2_target);

    Kinematics::velocities current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);

    //pass velocities to publisher object
    // current_vel.linear_x;
    // current_vel.linear_y;
    // current_vel.angular_z;

    // if (current_vel.linear_x != 0 || current_vel.angular_z != 0)
    // {
    //     Serial.print(current_vel.linear_x);
    //     Serial.print("\t");
    //     Serial.print(current_vel.angular_z);
    //     Serial.print("\t");
    //     Serial.println();
    // }
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}