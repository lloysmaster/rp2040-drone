#ifndef pid_H
#define pid_H

#include "pico/stdlib.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float error_previo;
    float integral;
    float limite_integral; // Anti-windup
} pid_axis_t;

// Inicializa o resetea los valores internos del PID
void pid_init(pid_axis_t *pid, float kp, float ki, float kd, float limite_i);

// Calcula la salida del PID
float pid_update(pid_axis_t *pid, float setpoint, float lectura_actual);

// Resetea solo la parte acumulada (integral y error previo)
void pid_reset(pid_axis_t *pid);

#endif