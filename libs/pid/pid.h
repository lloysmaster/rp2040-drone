#ifndef PID_H
#define PID_H

#include "pico/stdlib.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float lectura_previa;   // D-on-measurement
    float limite_integral;
    uint32_t last_time;
} pid_axis_t;

void pid_init(pid_axis_t *pid, float kp, float ki, float kd, float limite_i);
float pid_update(pid_axis_t *pid, float setpoint, float lectura_actual);
void pid_reset(pid_axis_t *pid);

#endif