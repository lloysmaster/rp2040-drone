#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>
#include "config.h"

typedef struct {
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} motor_output_t;

/**
 * @brief Mezcla las correcciones PID con el throttle y aplica límites de seguridad.
 * * @param throttle Valor base de potencia (0-2000)
 * @param corr_roll Corrección proveniente del PID de Roll
 * @param corr_pitch Corrección proveniente del PID de Pitch
 * @param corr_yaw Corrección proveniente del PID de Yaw
 * @return motor_output_t Estructura con los 4 valores finales para DShot
 */
motor_output_t mixer_compute(uint16_t throttle, float corr_roll, float corr_pitch, float corr_yaw);

#endif