#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "pico/stdlib.h"

typedef struct {
    // Actitud y Sensores
    float roll;
    float pitch;
    float yaw;
    
    // Motores (DShot values)
    uint16_t motors[4];
    
    // Radio y Control
    uint16_t throttle;
    
    // PID Status (opcional, para debug)
    float pid_p;
    float pid_i;
    float pid_d;
} telemetry_data_t;

// Inicializa el sistema de telemetría (si fuera necesario en el futuro)
void telemetry_init();

// Envía los datos por el puerto serie
void telemetry_send(const telemetry_data_t *data);

#endif