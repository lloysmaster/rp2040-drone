#include "mixer.h"

static uint16_t constrain_motor(int16_t value, uint16_t throttle) {
    if (throttle < IDLE_THRESHOLD) {
        return 0;
    }
    if (value < DSHOT_MIN) return DSHOT_MIN;
    if (value > DSHOT_MAX) return DSHOT_MAX;
    return (uint16_t)value;
}
static uint16_t clamp16(float v) {
    if (v < DSHOT_MIN) return DSHOT_MIN;
    if (v > DSHOT_MAX) return DSHOT_MAX;
    return (uint16_t)v;
}
motor_output_t mixer_compute(uint16_t throttle, float corr_roll, float corr_pitch, float corr_yaw) {
    motor_output_t out;

    if (throttle < IDLE_THRESHOLD) {
        out.m1 = out.m2 = out.m3 = out.m4 = 0;
        return out;
    }

    float t = (float)throttle;
    float m[4];
    m[0] = t - corr_roll - corr_pitch + corr_yaw;  // FR
    m[1] = t + corr_roll - corr_pitch - corr_yaw;  // FL
    m[2] = t - corr_roll + corr_pitch - corr_yaw;  // RR
    m[3] = t + corr_roll + corr_pitch + corr_yaw;  // RL

    // Escalar si algún motor se sale del rango
    float max_out = m[0];
    float min_out = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] > max_out) max_out = m[i];
        if (m[i] < min_out) min_out = m[i];
    }

    if (max_out > DSHOT_MAX) {
        float excess = max_out - DSHOT_MAX;
        for (int i = 0; i < 4; i++) m[i] -= excess;
    }
    if (min_out < DSHOT_MIN) {
        float deficit = DSHOT_MIN - min_out;
        for (int i = 0; i < 4; i++) m[i] += deficit;
    }

    out.m1 = clamp16(m[0]);
    out.m2 = clamp16(m[1]);
    out.m3 = clamp16(m[2]);
    out.m4 = clamp16(m[3]);

    return out;
}