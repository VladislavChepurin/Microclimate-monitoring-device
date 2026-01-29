// pid.h
#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd);
void PID_SetOutputLimits(PID_HandleTypeDef *pid, float min, float max);
float PID_Compute(PID_HandleTypeDef *pid, float input, float setpoint);
void PID_Reset(PID_HandleTypeDef *pid);

#endif /* __PID_H */
