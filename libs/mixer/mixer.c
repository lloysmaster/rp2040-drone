#include "mixer.h"

static uint16_t constrain_motor(int16_t value, uint16_t throttle) {
    if (throttle < MOTOR_IDLE_THRESHOLD) {
        return 0;
    }
    if (value < DSHOT_MIN_THROTTLE) return DSHOT_MIN_THROTTLE;
    if (value > DSHOT_MAX_THROTTLE) return DSHOT_MAX_THROTTLE;
    return (uint16_t)value;
}

motor_output_t mixer_compute(uint16_t throttle, float corr_roll, float corr_pitch, float corr_yaw) {
    motor_output_t out;

    // Mezcla según configuración en X
    // Motor 1: Front Right | Motor 2: Front Left | Motor 3: Rear Right | Motor 4: Rear Left
    int16_t m1_raw = throttle - (int16_t)corr_roll - (int16_t)corr_pitch + (int16_t)corr_yaw;
    int16_t m2_raw = throttle + (int16_t)corr_roll - (int16_t)corr_pitch - (int16_t)corr_yaw;
    int16_t m3_raw = throttle - (int16_t)corr_roll + (int16_t)corr_pitch - (int16_t)corr_yaw;
    int16_t m4_raw = throttle + (int16_t)corr_roll + (int16_t)corr_pitch + (int16_t)corr_yaw;

    // Aplicar límites y seguridad
    out.m1 = constrain_motor(m1_raw, throttle);
    out.m2 = constrain_motor(m2_raw, throttle);
    out.m3 = constrain_motor(m3_raw, throttle);
    out.m4 = constrain_motor(m4_raw, throttle);

    return out;
}