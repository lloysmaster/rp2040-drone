#include "pid.h"

void pid_init(pid_axis_t *pid, float kp, float ki, float kd, float limite_i) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->limite_integral = limite_i;
    pid_reset(pid);
}

void pid_reset(pid_axis_t *pid) {
    pid->integral      = 0.0f;
    pid->lectura_previa = 0.0f;
    pid->last_time     = time_us_32();
}

float pid_update(pid_axis_t *pid, float setpoint, float lectura_actual) {
    uint32_t now = time_us_32();
    float dt = (now - pid->last_time) / 1000000.0f;
    pid->last_time = now;

    // Protección contra dt inválido (primer ciclo o jitter extremo)
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;

    float error = setpoint - lectura_actual;

    // Integral sobre error puro, ki se aplica al final
    pid->integral += error * dt;
    if      (pid->integral >  pid->limite_integral) pid->integral =  pid->limite_integral;
    else if (pid->integral < -pid->limite_integral) pid->integral = -pid->limite_integral;

    // Derivativo sobre la medición (evita derivative kick al mover sticks)
    float d_term = -pid->kd * (lectura_actual - pid->lectura_previa) / dt;
    pid->lectura_previa = lectura_actual;

    return (pid->kp * error) + (pid->ki * pid->integral) + d_term;
}