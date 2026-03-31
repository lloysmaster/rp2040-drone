#include "pid.h"

void pid_init(pid_axis_t *pid, float kp, float ki, float kd, float limite_i) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->limite_integral = limite_i;
    pid_reset(pid);
}

void pid_reset(pid_axis_t *pid) {
    pid->error_previo = 0.0f;
    pid->integral = 0.0f;
}

float pid_update(pid_axis_t *pid, float setpoint, float lectura_actual) {
    float error = setpoint - lectura_actual;

    // Término Integral con anti-windup configurable
    pid->integral += pid->ki * error;
    if (pid->integral > pid->limite_integral) 
        pid->integral = pid->limite_integral;
    else if (pid->integral < -pid->limite_integral) 
        pid->integral = -pid->limite_integral;

    // Término Derivativo
    float d_term = pid->kd * (error - pid->error_previo);
    pid->error_previo = error;

    return (pid->kp * error) + pid->integral + d_term;
}