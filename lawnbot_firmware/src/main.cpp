#include <Arduino.h>

// uROS related
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

// std
#include <stdio.h>

// external libs
#include <odometry.h>

// custom
#include "motorController.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This code expects Arduino framework with serial transport
#endif

#define RCCHECK(fn){rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("error: ");Serial.println(temp_rc);Serial.println(RCL_RET_OK);error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define LED_PIN 2
// #define LAWNBOT
#ifndef LAWNBOT
// pin declaration

// Front Left wheel
int8_t FL_FORW = 35;
int8_t FL_BACK = 34;
int8_t FL_encoderPin = 18;

// Rear Left wheel
int8_t RL_FORW = 35;
int8_t RL_BACK = 34;
int8_t RL_encoderPin = 18;

// Front Right wheel
int8_t FR_FORW = 26;
int8_t FR_BACK = 25;
int8_t FR_encoderPin = 19;
// Rear Right wheel
int8_t RR_FORW = 35;
int8_t RR_BACK = 34;
int8_t RR_encoderPin = 18;

// parameters of the robot
float wheels_y_distance_ = 0.38;
float wheel_radius = 0.10;
float wheel_circumference_ = 2 * 3.14 * wheel_radius;

// encoder value per revolution of left wheel and right wheel
int tickPerRevolution_FLW = 128;
int tickPerRevolution_RLW = 128;
int tickPerRevolution_FRW = 128;
int tickPerRevolution_RRW = 128;

int threshold = 0; // TBD later depending on motor

// não se q porra é essa, dps mexo
// pid constants of front left wheel
float kp_fl = 1.8;
float ki_fl = 5;
float kd_fl = 0.1;
// pid constants of rear left wheel
float kp_rl = 1.8;
float ki_rl = 5;
float kd_rl = 0.1;
// pid constants of front right wheel
float kp_fr = 2.25;
float ki_fr = 5;
float kd_fr = 0.1;
// pid constants of rear right wheel
float kp_rr = 2.25;
float ki_rr = 5;
float kd_rr = 0.1;
#else
// pin declaration

// Left wheel
int8_t L_FORW = 26;
int8_t L_BACK = 14;
int8_t L_encoderPin = 27;

// right wheel
int8_t R_FORW = 3;
int8_t R_BACK = 1;
int8_t R_encoderPin = 16;

// parameters of the robot
float wheels_y_distance_ = 0.1;
float wheel_radius = 0.02;
float wheel_circumference_ = 2 * 3.14 * wheel_radius;

// encoder value per revolution of left wheel and right wheel
int tickPerRevolution_LW = 128;
int tickPerRevolution_RW = 128;

int threshold = 0; // TBD later depending on motor

// pid constants of left wheel
float kp_l = 1.8;
float ki_l = 5;
float kd_l = 0.1;
// pid constants of right wheel
float kp_r = 2.25;
float ki_r = 5;
float kd_r = 0.1;
#endif

// pwm parameters setup 
// não sei direito oq é isso, dps mexo
const int freq = 30000;
const int pwmChannelFLForward = 0;
const int pwmChannelFLBackward = 1;

const int pwmChannelRLForward = 0;
const int pwmChannelRLBackward = 1;

const int pwmChannelFRForward = 2;
const int pwmChannelFRBackward = 3;

const int pwmChannelRRForward = 2;
const int pwmChannelRRBackward = 3;

const int resolution = 8;


rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rcl_publisher_t odom_publisher;
std_msgs__msg__Int32 encodervalue_fl;
std_msgs__msg__Int32 encodervalue_rl;
std_msgs__msg__Int32 encodervalue_fr;
std_msgs__msg__Int32 encodervalue_rr;
nav_msgs__msg__Odometry odom_msg;

rcl_timer_t timer;
rcl_timer_t ControlTimer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

Odometry odometry;

// creating objects for right wheel and left wheel
MotorController frontLeftWheel(FL_FORW, FL_BACK, FL_encoderPin, tickPerRevolution_FLW);
MotorController rearLeftWheel(RL_FORW, RL_BACK, RL_encoderPin, tickPerRevolution_RLW);
MotorController frontRightWheel(FR_FORW, FR_BACK, FR_encoderPin, tickPerRevolution_FRW);
MotorController rearRightWheel(RR_FORW, RR_BACK, RR_encoderPin, tickPerRevolution_RRW);

void error_loop(rcl_ret_t returnCode)
{
    Serial.print("Error in ROS with return code: ");
    Serial.println(returnCode);
    while(1)
    {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(500);
    }
}

// interrupt function for left wheel encoder.
void updateEncoderFL()
{
  frontLeftWheel.EncoderCount.data++;
  encodervalue_fl = frontLeftWheel.EncoderCount;
}

// interrupt function for left wheel encoder.
void updateEncoderRL()
{
  rearLeftWheel.EncoderCount.data++;
  encodervalue_rl = rearLeftWheel.EncoderCount;
}


// interrupt function for right wheel encoder
void updateEncoderFR()
{
  frontRightWheel.EncoderCount.data++;
  encodervalue_fr = frontRightWheel.EncoderCount;
}

// interrupt function for right wheel encoder
void updateEncoderRR()
{
  rearRightWheel.EncoderCount.data++;
  encodervalue_rr = rearRightWheel.EncoderCount;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// function which publishes wheel odometry.
void publishData()
{
    odom_msg = odometry.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

// function which controlles the motor
void MotorControll_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    float linearVelocity;
    float angularVelocity;

    // linear velocity and angular velocity send cmd_vel topic
    linearVelocity = msg.linear.x;
    angularVelocity = msg.angular.z;

    // linear and angular velocities are converted to leftwheel and rightwheel velocities
    float vL = (linearVelocity - (angularVelocity * 1 / 2)) * 20;
    float vR = (linearVelocity + (angularVelocity * 1 / 2)) * 20;

    // current wheel rpm is calculated
    float currentRpmFL = frontLeftWheel.getRpm();
    float currentRpmRL = rearLeftWheel.getRpm();
    float currentRpmFR = frontRightWheel.getRpm();
    float currentRpmRR = rearRightWheel.getRpm();

    // pid controlled is used for generating the pwm signal
    float actuating_signal_FLW = frontLeftWheel.pid(vL, currentRpmFL);
    float actuating_signal_RLW = rearLeftWheel.pid(vL, currentRpmRL);
    float actuating_signal_FRW = frontRightWheel.pid(vR, currentRpmFR);
    float actuating_signal_RRW = rearRightWheel.pid(vR, currentRpmRR);
    if (vL == 0 && vR == 0)
    {
      frontLeftWheel.stop();
      rearLeftWheel.stop();
      frontRightWheel.stop();
      rearRightWheel.stop();
    }
    else
    {
      frontLeftWheel.moveBase(actuating_signal_FLW, pwmChannelFLForward, pwmChannelFLBackward);
      rearLeftWheel.moveBase(actuating_signal_RLW, pwmChannelRLForward, pwmChannelRLBackward);
      frontRightWheel.moveBase(actuating_signal_FRW, pwmChannelFRForward, pwmChannelFRBackward);
      rearRightWheel.moveBase(actuating_signal_RRW, pwmChannelRRForward, pwmChannelRRBackward);
    }
    // odometry
    // mexer nisso com calma
    float average_rps_x = ((float)(currentRpmL + currentRpmR) / 2) / 60.0; // RPM
    float linear_x = average_rps_x * wheel_circumference_;                 // m/s
    float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 60.0;
    float angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s
    float linear_y = 0;
    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    
    odometry.update(
        vel_dt,
        linear_x,
        linear_y,
        angular_z);
    publishData();
}

void subscription_callback(const void *msgin)
{
    prev_cmd_time = millis();
}

void configureGPIO()
{
  // perguntar pro matheus o que são esses parametros
  // initializing the pid constants
  frontLeftWheel.initPID(kp_fl, ki_fl, kd_fl);
  rearLeftWheel.initPID(kp_rl, ki_rl, kd_rl);
  frontRightWheel.initPID(kp_fr, ki_fr, kd_fr);
  rearRightWheel.initPID(kp_rr, ki_rr, kd_rr);

  // initializing interrupt functions for counting the encoder tick values
  attachInterrupt(digitalPinToInterrupt(frontLeftWheel.g_EncoderPinA), updateEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(rearLeftWheel.g_EncoderPinA), updateEncoderRL, RISING);
  attachInterrupt(digitalPinToInterrupt(frontRightWheel.g_EncoderPinA), updateEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(rearRightWheel.g_EncoderPinA), updateEncoderRR, RISING);

  // Initialize PWM signal parameters
  // Front Left Wheel
  ledcSetup(pwmChannelFLForward, freq, resolution);
  ledcAttachPin(frontLeftWheel.g_Forward, pwmChannelFLForward); // Use Forward pin for PWM
  ledcSetup(pwmChannelFLBackward, freq, resolution);
  ledcAttachPin(frontLeftWheel.g_Backward, pwmChannelFLBackward); // Use Backward pin for PWMedcSetup(pwmChannelLForward, freq, resolution);
  
  // Rear Left Wheel
  ledcSetup(pwmChannelRLForward, freq, resolution);
  ledcAttachPin(rearLeftWheel.g_Forward, pwmChannelRLForward); // Use Forward pin for PWM
  ledcSetup(pwmChannelRLBackward, freq, resolution);
  ledcAttachPin(rearLeftWheel.g_Backward, pwmChannelRLBackward); // Use Backward pin for PWM
 
  // Front Right Wheel
  ledcSetup(pwmChannelFRForward, freq, resolution);
  ledcAttachPin(frontRightWheel.g_Forward, pwmChannelFRForward); // Use Forward pin for PWM
  ledcSetup(pwmChannelFRBackward, freq, resolution);
  ledcAttachPin(frontRightWheel.g_Backward, pwmChannelFRBackward); // Use Backward pin for PWM

  // Rear Right Wheel
  ledcSetup(pwmChannelRRForward, freq, resolution);
  ledcAttachPin(rearRightWheel.g_Forward, pwmChannelRRForward); // Use Forward pin for PWM
  ledcSetup(pwmChannelRRBackward, freq, resolution);
  ledcAttachPin(rearRightWheel.g_Backward, pwmChannelRRBackward); // Use Backward pin for PWM
  
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
}

void setup()
{
  
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  Serial.println("Serial transport initialized");

  allocator = rcl_get_default_allocator();
  
  Serial.println("Initialized ROS allocator");

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  

  Serial.println("Initialized ROS support init options");


  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));
  Serial.println("uROS node created");

  
  // create subscriber for cmd_vel topic
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  Serial.println("/cmdvel subscriber created");

  
  // create a odometry publisher
  RCCHECK(rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom/unfiltered"));
  
  Serial.println("Odom publisher created");

  // timer function for controlling the motor base. At every samplingT time
  // MotorControll_callback function is called
  // Here I had set SamplingT=10 Which means at every 10 milliseconds MotorControll_callback function is called
  const unsigned int samplingT = 10;
  RCCHECK(rclc_timer_init_default(
      &ControlTimer,
      &support,
      RCL_MS_TO_NS(samplingT),
      MotorControll_callback));
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));

  configureGPIO();
  Serial.println("-----------------end setup--------------------");
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}


