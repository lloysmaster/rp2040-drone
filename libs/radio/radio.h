#ifndef radio_H
#define radio_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

#define NUM_CHANNELS 8
#define SYNC_THRESHOLD 2700

// Estructura para manejar la instancia del receptor
typedef struct {
    PIO pio;
    uint sm;
    uint32_t raw_values[NUM_CHANNELS];
    int current_channel;
} ppm_rx_t;

// Inicializa el PIO y la estructura de datos
void radio_init(ppm_rx_t *rx, PIO pio, uint sm, uint pin);

// Procesa el FIFO del PIO y actualiza los valores de los canales
void radio_update(ppm_rx_t *rx);

// Retorna el valor de un canal específico (1000-2000ms aprox)
uint32_t radio_get_channel(ppm_rx_t *rx, uint8_t channel);

#endif