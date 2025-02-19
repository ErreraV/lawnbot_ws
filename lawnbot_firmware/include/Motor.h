#pragma once 
#include <stdlib.h>
#include <Arduino.h>
#define ticks_per_rev 36

class Motor {

private:
    uint8_t m_cw_bit;
    uint8_t m_ccw_bit;

    int m_encoder_pin;
    int m_pwm_pin;
    int m_pwm_channel;

    int m_encoder_value;
    float m_rpm;

    volatile long m_current_position;
    volatile long m_previous_position;
    volatile long m_current_time;
    volatile long m_previous_time;

    // PID variables
    float rpmFilt;
    float eintegral;
    float ederivative;
    float rpmPrev;
    float kp;
    float ki;
    float kd;
    float error;
    float previousError = 0;
    volatile long CurrentTimeforError;
    volatile long PreviousTimeForError;
public:
    Motor(uint8_t cw_bit, uint8_t ccw_bit,int encoder_pin, int pwm_pin, int pwm_channel);

    void calculateVelocity();
    void initPID(float proportionalGain, float integralGain, float derivativeGain);
    float pid(float setpoint, float feedback);

    inline float getRPM() { calculateVelocity(); return m_rpm;}
    void incrementEncoderData() {m_encoder_value++;}

    void stop();
    void move(float signal);
};