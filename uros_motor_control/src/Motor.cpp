#include "motorController.h"
#include <Arduino.h>

MotorController::MotorController(int8_t ForwardPin, int8_t BackwardPin, int8_t EncoderA, int tickPerRevolution)
{
    g_Forward = ForwardPin;
    g_Backward = BackwardPin;
    g_EncoderPinA = EncoderA;
    m_ticksPerRevolution = tickPerRevolution;
    pinMode(g_Forward, OUTPUT);
    pinMode(g_Backward, OUTPUT);
    pinMode(g_EncoderPinA, INPUT);
}

void MotorController::initPID(float proportionalGain, float integralGain, float derivativeGain)
{
    kp = proportionalGain;
    ki = integralGain;
    kd = derivativeGain;
}

float MotorController::getRpm()
{
    CurrentPosition = EncoderCount.data;
    CurrentTime = millis();
    float delta1 = ((float)CurrentTime - PreviousTime) / 1.0e3;
    float velocity = ((float)CurrentPosition - PreviousPosition) / delta1;
    float rpm = (velocity / m_ticksPerRevolution) * 60;
    rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;
    float rpmPrev = rpm;
    PreviousPosition = CurrentPosition;
    PreviousTime = CurrentTime;
    // Serial.println(rpmFilt);
    return rpmFilt;
}

float MotorController::pid(float setpoint, float feedback)
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

void MotorController::moveBase(float ActuatingSignal, int pwmChannelForward, int pwmChannelBackward)
{
    int pwm = (int)fabs(ActuatingSignal);
    if (pwm > 255)
        pwm = 255;

    if (ActuatingSignal > 0)
    {
        ledcWrite(pwmChannelForward, pwm);
        ledcWrite(pwmChannelBackward, 0);
    }
    else
    {
        ledcWrite(pwmChannelForward, 0);
        ledcWrite(pwmChannelBackward, pwm);
    }
}

void MotorController::stop()
{
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
}