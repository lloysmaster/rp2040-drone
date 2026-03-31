#include "dshot.h"
#include "pio_dshot.pio.h"
#include <stdbool.h>

void dshot_init(dshot_config_t *config, PIO pio, uint base_pin) {
    config->pio = pio;
    uint offset = pio_add_program(pio, &dshot300_program);

    for (int i = 0; i < 4; i++) {
        config->sm[i] = i; // Usamos las 4 SM del bloque PIO seleccionado
        dshot300_program_init(config->pio, config->sm[i], offset, base_pin + i);
    }

    // Fase de armado inicial: enviar ceros durante un tiempo
    for (int i = 0; i < 100; i++) {
        dshot_send_all(config, 0, 0, 0, 0);
        sleep_ms(10);
    }
}

uint16_t dshot_prepare_packet(uint16_t value, bool telemetry) {
    // 1. Crear el paquete base: [valor (11 bits) | telemetría (1 bit)]
    uint16_t packet = (value << 1) | (telemetry ? 1 : 0);
    
    // 2. Calcular Checksum (XOR de los 3 nibbles de 4 bits del paquete de 12 bits)
    uint16_t checksum = 0;
    uint16_t temp_packet = packet;
    
    for (int i = 0; i < 3; i++) {
        checksum ^= (temp_packet & 0x0F);
        temp_packet >>= 4;
    }
    checksum &= 0x0F;

    // 3. Unir: [paquete (12 bits) | checksum (4 bits)] = 16 bits totales
    return (packet << 4) | checksum;
}

void dshot_send_all(dshot_config_t *config, uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    uint16_t values[4] = {m1, m2, m3, m4};
    for (int i = 0; i < 4; i++) {
        // Para comandos (1-47), activamos telemetría (true)
        // Para aceleración (>= 48), telemetría en false (o según necesites)
        bool is_command = (values[i] > 0 && values[i] < 48);
        
        uint16_t packet = dshot_prepare_packet(values[i], is_command);
        
        // Enviar al PIO (alineado a la izquierda para el FIFO de 32 bits)
        pio_sm_put_blocking(config->pio, config->sm[i], (uint32_t)packet << 16);
    }
}