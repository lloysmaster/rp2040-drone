#ifndef DSHOT_H
#define DSHOT_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pio_ppm_rx.pio.h"
#include "config.h"


typedef struct {
    PIO pio;
    uint sm[4];
} dshot_config_t;

// Inicializa el PIO y los 4 motores comenzando desde base_pin
void dshot_init(dshot_config_t *config, PIO pio, uint base_pin);

// Envía el comando a los 4 motores simultáneamente
void dshot_send_all(dshot_config_t *config, uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

// Función interna para preparar el paquete (útil si necesitas telemetría luego)
uint16_t dshot_prepare_packet(uint16_t value, bool telemetry);

#endif