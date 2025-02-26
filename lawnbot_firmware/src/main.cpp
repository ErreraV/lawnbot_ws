#include <Arduino.h>

// uROS related
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

// std
#include <stdio.h>

// external libs
#include <odometry.h>

// custom
#include "Motor.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This code expects Arduino framework with serial transport
#endif

#define RCCHECK(fn){rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("error: ");Serial.println(temp_rc);Serial.println(RCL_RET_OK);error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// pin declaration
#define SerialPin 21
#define ClockPin 19
#define LatchPin 23
#define EnablePin 18

// Pinos Encoder
#define EncoderM1Pin 26
#define EncoderM2Pin 27
#define EncoderM3Pin 14
#define EncoderM4Pin 12

#define MotorOff 0
#define m4_cw  1
#define m2_cw  2
#define m1_cw  4
#define m1_ccw 8
#define m2_ccw 16
#define m3_cw  32
#define m4_ccw 64
#define m3_ccw 128

// Defina os pinos PWM
#define PWM1Pin 22
#define PWM2Pin 32
#define PWM3Pin 33
#define PWM4Pin 25

// Defina os canais PWM
#define PWM1Channel 0
#define PWM2Channel 1
#define PWM3Channel 2
#define PWM4Channel 3

// Defina o pino da lamina

#define BladePin 13

// Defina a frequência PWM e a resolução
#define PWMFrequency 30000
#define PWMResolution 8


// parameters of the robot  
float wheels_y_distance_ = 0.31;
float wheel_radius = 0.07;
float wheel_circumference_ = 2 * 3.14 * wheel_radius;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_subscription_t blade_subscriber;
std_msgs__msg__Int32 blade_msg;

rcl_publisher_t odom_publisher;
rcl_publisher_t debug_publisher;
std_msgs__msg__String debug;
std_msgs__msg__Int32 encodervalue_l;
std_msgs__msg__Int32 encodervalue_r;
nav_msgs__msg__Odometry odom_msg;

rcl_timer_t timer;
rcl_timer_t ControlTimer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

Odometry odometry;
char debug_buffer[50];

Motor M1_wheel(m1_cw, m1_ccw, EncoderM1Pin, PWM1Pin, PWM1Channel);
Motor M2_wheel(m2_cw, m2_ccw, EncoderM2Pin, PWM2Pin, PWM2Channel);
Motor M3_wheel(m3_cw, m3_ccw, EncoderM3Pin, PWM3Pin, PWM3Channel);
Motor M4_wheel(m4_cw, m4_ccw, EncoderM4Pin, PWM4Pin, PWM4Channel);


void pulse(uint8_t pin){
	digitalWrite(pin, HIGH);
	delay(1);
	digitalWrite(pin, LOW);
	delay(1);
}

void writeMotorDir(uint8_t control){

  for(uint8_t i = 0; i < 8; ++i){
    digitalWrite(SerialPin, control & 0x80 ? HIGH : LOW);
    pulse(ClockPin);
    control <<= 1;
  }
  pulse(LatchPin);  
}

void error_loop(rcl_ret_t returnCode)
{
    Serial.print("Error in ROS with return code: ");
    Serial.println(returnCode);
    while(1)
    {
        delay(500);
    }
}

void encoderM1InterruptionHandler(){
  M1_wheel.incrementEncoderData();
}
void encoderM2InterruptionHandler(){
  M2_wheel.incrementEncoderData();
}
void encoderM3InterruptionHandler(){
  M3_wheel.incrementEncoderData();
}
void encoderM4InterruptionHandler(){
  M4_wheel.incrementEncoderData();
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

void publishDebug()
{
    debug.data.data = debug_buffer;
    debug.data.size = strlen(debug_buffer);
    debug.data.capacity = strlen(debug_buffer) + 1;
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug, NULL));
}

float getSignal(float velocity){
  float signal = 255 * velocity / 0.73; // 0.73 eh a velocidade maxima do motor
  return signal;
}

// function which controlles the motor
void MotorControll_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // const geometry_msgs__msg__Twist *cmd_vel_msg = (const geometry_msgs__msg__Twist *)msgin;

    float linearVelocity;
    float angularVelocity;
    float acceleration_factor = 0.2;

    // linear velocity and angular velocity send cmd_vel topic
    linearVelocity = cmd_vel_msg.linear.x;
    angularVelocity = cmd_vel_msg.angular.z;

    // linear and angular velocities are converted to leftwheel and rightwheel velocities
    float vL = (linearVelocity - (angularVelocity * 1 / 2)) * 20; //alterar isso so n sei o valor
    float vR = (linearVelocity + (angularVelocity * 1 / 2)) * 20;

    // current wheel rpm is calculated
    float currentVelocityRM1 = M1_wheel.getRPM() * wheel_circumference_ / 60;
    float currentVelocityRM2 = M2_wheel.getRPM() * wheel_circumference_ / 60;
    float currentVelocityLM3 = M3_wheel.getRPM() * wheel_circumference_ / 60;
    float currentVelocityLM4 = M4_wheel.getRPM() * wheel_circumference_ / 60;

    float currentSignalRM1 = getSignal(currentVelocityRM1);
    float currentSignalRM2 = getSignal(currentVelocityRM2);
    float currentSignalLM3 = getSignal(currentVelocityLM3);
    float currentSignalLM4 = getSignal(currentVelocityLM4);

    float targetSignalR = getSignal(vR);
    float targetSignalL = getSignal(vL);
    
    while (fabs(currentSignalRM1 - targetSignalR) > 1.0 || fabs(currentSignalLM3 - targetSignalL) > 1.0) {
        // Ajusta a velocidade atual em direção à velocidade desejada
        currentSignalRM1 += acceleration_factor * (targetSignalR - currentSignalRM1);
        currentSignalLM3 += acceleration_factor * (targetSignalL - currentSignalLM3);

        // Atualiza a direção dos motores
        u_int8_t control = MotorOff;

        if (currentSignalRM1 > 0.0) {
            control += m1_cw + m2_cw;
        } else if (currentSignalRM1 < 0.0) {
            control += m1_ccw + m2_ccw;
        }

        if (currentSignalLM3 > 0) {
            control += m3_cw + m4_cw;
        } else if (currentSignalLM3 < 0) {
            control += m3_ccw + m4_ccw;
        }

        writeMotorDir(control);
        M1_wheel.move(currentSignalRM1);
        M2_wheel.move(currentSignalRM1);
        M3_wheel.move(currentSignalLM3);
        M4_wheel.move(currentSignalLM3);

        // Atualiza as velocidades atuais
        currentVelocityRM1 = M1_wheel.getRPM() * wheel_circumference_ / 60;
        currentVelocityLM3 = M3_wheel.getRPM() * wheel_circumference_ / 60;
        currentSignalRM1 = getSignal(currentVelocityRM1);
        currentSignalLM3 = getSignal(currentVelocityLM3);
    }

    // odometry
    //float average_rps_x = ((float)(currentRpmRM1 + currentRpmRM2 + currentRpmLM3 + currentRpmLM4) / 4) / 60.0; // RPM
    //float linear_x = average_rps_x * wheel_circumference_;                 // m/s
    // float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 60.0;
    // float angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s
    // float linear_y = 0;
    // unsigned long now = millis();
    // float vel_dt = (now - prev_odom_update) / 1000.0;
    // prev_odom_update = now;
    
    // odometry.update(
    //     vel_dt,
    //     linear_x,
    //     linear_y,
    //     angular_z);
    // publishData();
}

void subscription_callback(const void *msgin)
{
    prev_cmd_time = millis();
}

void blade_callback(const void * msgin){
  const std_msgs__msg__Int32 * blade_msg = (const std_msgs__msg__Int32 *)msgin;
  if(blade_msg->data == 1){
    digitalWrite(BladePin, HIGH);
  } else {
    digitalWrite(BladePin, LOW);
  }
  sprintf(debug_buffer, "Blade state: %d", blade_msg->data);
  publishDebug();
}

void configureGPIO()
{

  pinMode(SerialPin, OUTPUT);
  pinMode(ClockPin, OUTPUT);
  pinMode(LatchPin, OUTPUT);
  pinMode(EnablePin, OUTPUT);
  pinMode(BladePin, OUTPUT);

  // Configure os canais PWM
  ledcSetup(PWM1Channel, PWMFrequency, PWMResolution);
  ledcSetup(PWM2Channel, PWMFrequency, PWMResolution);
  ledcSetup(PWM3Channel, PWMFrequency, PWMResolution);
  ledcSetup(PWM4Channel, PWMFrequency, PWMResolution);

  // Anexe os pinos PWM aos canais
  ledcAttachPin(PWM1Pin, PWM1Channel);
  ledcAttachPin(PWM2Pin, PWM2Channel);
  ledcAttachPin(PWM3Pin, PWM3Channel);
  ledcAttachPin(PWM4Pin, PWM4Channel);


  
  attachInterrupt(digitalPinToInterrupt(EncoderM1Pin), encoderM1InterruptionHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderM2Pin), encoderM2InterruptionHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderM3Pin), encoderM3InterruptionHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderM4Pin), encoderM4InterruptionHandler, RISING);

  digitalWrite(EnablePin, LOW);
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

  // create init_optionodometrys
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

  RCCHECK(rclc_subscription_init_default(
    &blade_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/blade_state"));

  Serial.println("/cmdvel subscriber created");

  
  // create a odometry publisher
  RCCHECK(rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom/unfiltered"));
  
  Serial.println("Odom publisher created");

  RCCHECK(rclc_publisher_init_default(
      &debug_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "debug"));
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
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &blade_subscriber, &blade_msg, blade_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));

  configureGPIO();
  Serial.println("-----------------end setup--------------------");
}

int count = 0; //tmp
u_int8_t duty_cycle = 64;
void loop()
{
  // put your main code here, to run repeatedly:
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
//   if(count % 1000 == 0){
//     ledcWrite(PWM5Cannel, duty_cycle);
//     duty_cycle = (duty_cycle == 64) ? 0 : 64;
//   }
//   count++;
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


