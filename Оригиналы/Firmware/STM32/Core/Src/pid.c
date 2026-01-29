/*
 * pid.c
 *
 *  Created on: Jan 27, 2026
 *      Author: chepu
 */

// pid.c
#include "pid.h"

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = 0.0f;
    pid->output_max = 1.0f;
}

void PID_SetOutputLimits(PID_HandleTypeDef *pid, float min, float max)
{
    pid->output_min = min;
    pid->output_max = max;
}

float PID_Compute(PID_HandleTypeDef *pid, float input, float setpoint)
{
    float error = setpoint - input;

    // Пропорциональная составляющая
    float P = pid->Kp * error;

    // Интегральная составляющая
    pid->integral += error;
    float I = pid->Ki * pid->integral;

    // Дифференциальная составляющая
    float D = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    // Суммирование
    float output = P + I + D;

    // Ограничение выхода
    if(output > pid->output_max)
        output = pid->output_max;
    else if(output < pid->output_min)
        output = pid->output_min;

    return output;
}

void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
