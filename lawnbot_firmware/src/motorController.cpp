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
// #define lawnbot
#ifndef LAWNBOT
// pin declaration

// Left wheel
int8_t L_FORW = 35;
int8_t L_BACK = 34;
int8_t L_encoderPin = 18;

// right wheel
int8_t R_FORW = 26;
int8_t R_BACK = 25;
int8_t R_encoderPin = 19;

// parameters of the robotA
float wheels_y_distance_ = 0.38;
float wheel_radius = 0.10;
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
const int freq = 30000;
const int pwmChannelLForward = 0;
const int pwmChannelLBackward = 1;
const int pwmChannelRForward = 2;
const int pwmChannelRBackward = 3;
const int resolution = 8;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rcl_publisher_t odom_publisher;
std_msgs__msg__Int32 encodervalue_l;
std_msgs__msg__Int32 encodervalue_r;
nav_msgs__msg__Odometry odom_msg;

rcl_timer_t timer;
rcl_timer_t ControlTimer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

Odometry odometry;

// creating objects for right wheel and left wheel
MotorController leftWheel(L_FORW, L_BACK, L_encoderPin, tickPerRevolution_LW);
MotorController rightWheel(R_FORW, R_BACK, R_encoderPin, tickPerRevolution_RW);

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
void updateEncoderL()
{
  leftWheel.EncoderCount.data++;
  encodervalue_l = leftWheel.EncoderCount;
}

// interrupt function for right wheel encoder
void updateEncoderR()
{
  rightWheel.EncoderCount.data++;
  encodervalue_r = rightWheel.EncoderCount;
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
    float currentRpmL = leftWheel.getRpm();
    float currentRpmR = rightWheel.getRpm();

    // pid controlled is used for generating the pwm signal
    float actuating_signal_LW = leftWheel.pid(vL, currentRpmL);
    float actuating_signal_RW = rightWheel.pid(vR, currentRpmR);

    if (vL == 0 && vR == 0)
    {
      leftWheel.stop();
      rightWheel.stop();
    }
    else
    {
      leftWheel.moveBase(actuating_signal_LW, pwmChannelLForward, pwmChannelLBackward);
      rightWheel.moveBase(actuating_signal_RW, pwmChannelRForward, pwmChannelRBackward);
    }
    // odometry
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

  // initializing the pid constants
  leftWheel.initPID(kp_l, ki_l, kd_l);
  rightWheel.initPID(kp_r, ki_r, kd_r);
  // initializing interrupt functions for counting the encoder tick values
  attachInterrupt(digitalPinToInterrupt(leftWheel.g_EncoderPinA), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheel.g_EncoderPinA), updateEncoderR, RISING);


  // Initialize PWM signal parameters
  ledcSetup(pwmChannelLForward, freq, resolution);
  ledcAttachPin(leftWheel.g_Forward, pwmChannelLForward); // Use Forward pin for PWM
  ledcSetup(pwmChannelLBackward, freq, resolution);
  ledcAttachPin(leftWheel.g_Backward, pwmChannelLBackward); // Use Backward pin for PWM
  ledcSetup(pwmChannelRForward, freq, resolution);
  ledcAttachPin(rightWheel.g_Forward, pwmChannelRForward); // Use Forward pin for PWM
  ledcSetup(pwmChannelRBackward, freq, resolution);
  ledcAttachPin(rightWheel.g_Backward, pwmChannelRBackward); // Use Backward pin for PWM

  
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
  const unsigned int samplingT = 20;
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


