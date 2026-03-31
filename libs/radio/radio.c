#include "../radio/radio.h"// El header generado por el compilador PIO
#include "pio_ppm_rx.pio.h"


void radio_init(ppm_rx_t *rx, PIO pio, uint sm, uint pin) {
    rx->pio = pio;
    rx->sm = sm;
    rx->current_channel = 0;

    for(int i = 0; i < NUM_CHANNELS; i++) {
        // Suponiendo que el canal 0 o el 2 es tu Throttle según tu main
        // Lo inicializamos en 0 para seguridad total
        if (i == 2) { 
            rx->raw_values[i] = 0; 
        } else {
            rx->raw_values[i] = 1500; // Neutro para Roll/Pitch/Yaw
        }
    }

    uint offset = pio_add_program(pio, &ppm_rx_program);
    pio_sm_config c = ppm_rx_program_get_default_config(offset);
    
    sm_config_set_in_pins(&c, pin);
    sm_config_set_clkdiv(&c, 62.5f); 
    
    pio_gpio_init(pio, pin);
    gpio_set_pulls(pin, true, false);
    
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void radio_update(ppm_rx_t *rx) {
    while (!pio_sm_is_rx_fifo_empty(rx->pio, rx->sm)) {
        uint32_t val = pio_sm_get(rx->pio, rx->sm);
        
        if (val > SYNC_THRESHOLD) {
            rx->current_channel = 0;
        } else if (rx->current_channel < NUM_CHANNELS) {
            // Filtro básico de rango para evitar ruidos
            if (val > 400 && val < 2400) {
                rx->raw_values[rx->current_channel] = val;
            }
            rx->current_channel++;
        }
    }
}

uint32_t radio_get_channel(ppm_rx_t *rx, uint8_t channel) {
    if (channel < NUM_CHANNELS) {
        return rx->raw_values[channel];
    }
    
    // Failsafe para canales inexistentes
    // Si es el canal de throttle (0 o 2 según uses), devuelve 0
    if (channel == 2) return 0;
    
    return 1500; // Neutro para el resto
}