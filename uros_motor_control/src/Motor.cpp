#include "Motor.h"

Motor::Motor(uint8_t cw_bit, uint8_t ccw_bit,int encoder_pin, int pwm_pin, int pwm_channel){
    m_cw_bit = cw_bit;
    m_ccw_bit = ccw_bit;
    m_encoder_pin = encoder_pin;
    m_pwm_pin = pwm_pin;
    m_pwm_channel = pwm_channel;
    m_current_position = 0;
    m_previous_position = 0;
    m_previous_time = millis();
    m_current_time = 0;
    m_rpm = 0;

    rpmFilt = 1;
    eintegral= 0;
    ederivative= 0;
    rpmPrev= 0;
    kp= 0;
    ki= 0;
    kd= 0;
    error= 0;
    previousError = 0;
    CurrentTimeforError = 0;
    PreviousTimeForError = millis();
    pinMode(m_encoder_pin, INPUT);
}

void Motor::initPID(float proportionalGain, float integralGain, float derivativeGain)
{
    kp = proportionalGain;
    ki = integralGain;
    kd = derivativeGain;
}

float Motor::pid(float setpoint, float feedback)
{
    CurrentTimeforError = millis();
    float delta2 = ((float)CurrentTimeforError - PreviousTimeForError) / 1.0e3;
    error = setpoint - feedback;
    eintegral = eintegral + (error * delta2);
    ederivative = (error - previousError) / delta2;
    float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

    previousError = error;
    PreviousTimeForError = CurrentTimeforError;
    return control_signal;
}

void Motor::calculateVelocity(){
    m_current_position = m_encoder_value;
    m_current_time = millis();

    float delta = ((float)m_current_time - m_previous_time) / 1.0e3;
    float velocity = ((float)m_current_position - m_previous_position) / delta;
    float rpm = (velocity / ticks_per_rev) * 60;
    //rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;

    //fazer pid dps
    //Serial.println(rpm);
    m_previous_time = m_current_time;
    m_previous_position = m_current_position;
    m_rpm = rpm;
}

void Motor::stop(){
    ledcWrite(m_pwm_channel, 0);
}

void Motor::move(float signal){
    int pwm = (int)fabs(signal);
    if (pwm > 255)
        pwm = 255;

    ledcWrite(m_pwm_channel, pwm);
}