#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// Archivos PIO generados (Asegúrate de tener ambos en tu CMakeLists.txt)
#include "pio_ppm_rx.pio.h"
#include "pio_dshot.pio.h"

// --- CONFIGURACIÓN PINES ---
#define PPM_PIN          2
#define MPU_SPI_INST     spi1
#define PIN_MISO         12
#define PIN_CS           13
#define PIN_SCK          10
#define PIN_MOSI         11
#define MOTOR_BASE_PIN   16 // Motores en 16, 17, 18, 19

// --- CONFIGURACIÓN MPU6500 ---
#define REG_GYRO_CONFIG   0x1B
#define REG_PWR_MGMT_1    0x6B
#define REG_GYRO_XOUT_H   0x43
#define READ_BIT          0x80

// --- VARIABLES RECEPCIÓN PPM ---
#define NUM_CHANNELS 8
#define SYNC_THRESHOLD 2700
uint32_t raw_values[NUM_CHANNELS];
int current_channel = 0;


// --- VARIABLES GLOBALES DE PRUEBA ---
uint16_t motor_test_val = 0; // Valor manual para el motor



// --- VARIABLES CONTROL Y PID ---
typedef struct {
    float kp, ki, kd;
    float error_previo, integral;
} pid_axis_t;

pid_axis_t pid_roll  = { .kp = 0.00f, .ki = 0.000f, .kd = 0.00f };
pid_axis_t pid_pitch = { .kp = 0.00f, .ki = 0.000f, .kd = 0.00f };
pid_axis_t pid_yaw   = { .kp = 0.00f, .ki = 0.000f, .kd = 0.00f };

// --- ESTRUCTURA DSHOT PARA 4 MOTORES ---
typedef struct {
    PIO pio;
    uint sm;
} motor_t;
motor_t motores[4];

// --- PROTOTIPOS ---
void init_hardware();
void process_ppm();
float calcular_pid(pid_axis_t *pid, float setpoint, float lectura_actual);
void send_dshot_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

// --- LÓGICA MPU6500 ---
static inline void cs_select() { gpio_put(PIN_CS, 0); }
static inline void cs_deselect() { gpio_put(PIN_CS, 1); }

void mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[] = { reg, data };
    cs_select();
    spi_write_blocking(MPU_SPI_INST, buf, 2);
    cs_deselect();
}

void mpu_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t reg = REG_GYRO_XOUT_H | READ_BIT;
    uint8_t buffer[6];
    cs_select();
    spi_write_blocking(MPU_SPI_INST, &reg, 1);
    spi_read_blocking(MPU_SPI_INST, 0, buffer, 6);
    cs_deselect();
    *gx = (buffer[0] << 8) | buffer[1];
    *gy = (buffer[2] << 8) | buffer[3];
    *gz = (buffer[4] << 8) | buffer[5];
}

// --- LÓGICA DSHOT ---
uint16_t dshot_prepare_packet(uint16_t value) {
    if (value == 0) return 0;
    uint16_t packet = (value << 1); 
    uint16_t checksum = 0;
    uint16_t temp_packet = packet;
    for (int i = 0; i < 3; i++) { checksum ^= (temp_packet & 0x0F); temp_packet >>= 4; }
    return (packet << 4) | (checksum & 0x0F);
}

void send_dshot_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    uint16_t vals[4] = {m1, m2, m3, m4};
    for(int i=0; i<4; i++) {
        uint16_t packet = dshot_prepare_packet(vals[i]);
        pio_sm_put_blocking(motores[i].pio, motores[i].sm, packet << 16);
    }
}
void check_console_input() {
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        static char buffer[32];
        static int idx = 0;

        if (c == '\n' || c == '\r') {
            buffer[idx] = '\0';
            if (idx > 0) {
                // Comandos: 'M100' para motor, 'P0.5' para Kp, etc.
                char type = buffer[0];
                float val = atof(&buffer[1]);

                if (type == 'M' || type == 'm') {
                    motor_test_val = (uint16_t)val;
                    printf("Motor Test Set: %d\n", motor_test_val);
                } else if (type == 'P' || type == 'p') {
                    pid_roll.kp = pid_pitch.kp = val;
                    printf("Kp actualizado a: %.4f\n", val);
                } else if (type == 'D' || type == 'd') {
                    pid_roll.kd = pid_pitch.kd = val;
                    printf("Kd actualizado a: %.4f\n", val);
                }
            }
            idx = 0;
        } else {
            if (idx < 31) buffer[idx++] = (char)c;
        }
    }
}



// --- MAIN ---
int main() {
    stdio_init_all();
    init_hardware();

    printf("Modo Test: Usa 'Mxxx' para motor (48-2000) o 'Pxxx' para Kp\n");

    int16_t gx, gy, gz;
    uint32_t target_loop_time_us = 2000; 
    uint32_t next_loop_time = time_us_32();

    while (true) {
        while (time_us_32() < next_loop_time);
        next_loop_time += target_loop_time_us;

        // 0. Leer consola (No bloqueante)
        check_console_input();

        // 1. Procesar Radio
        process_ppm();
        float throttle_input = raw_values[2]; 

        // 2. Leer Sensores
        mpu_read_gyro(&gx, &gy, &gz);
        float gyro_roll  = (float)gx / 16.4f;
        float gyro_pitch = (float)gy / 16.4f;

        // 3. Calcular PID
        float setpoint_roll = ((float)raw_values[0] - 1500) / 10.0f;
        float corr_roll = calcular_pid(&pid_roll, setpoint_roll, gyro_roll);
        
        // 4. Lógica de un solo motor para pruebas
        uint16_t base_throttle = 0;
        
        if (motor_test_val > 0) {
            // Si pusiste un valor por consola, ese es nuestro "piso"
            base_throttle = motor_test_val;
        } else {
            // Si no, usamos el control remoto
            base_throttle = (uint16_t)throttle_input;
        }

        // Sumamos la corrección PID al valor base
        // Ahora m_out reaccionará al giro incluso si motor_test_val está activo
        int16_t m_calculado = (int16_t)base_throttle - (int16_t)corr_roll;

        // 5. Seguridad y DShot
        uint16_t m_out = 0;
        if (base_throttle < 48) {
            m_out = 0; // Seguridad: si el base es 0, motor apagado
        } else {
            if (m_calculado < 48) m_out = 48;
            else if (m_calculado > 2000) m_out = 2000;
            else m_out = (uint16_t)m_calculado;
        }

        send_dshot_all(m_out, 0, 0, 0);
        
        // Telemetría rápida para ver qué pasa
        static int count = 0;
        if (count++ % 100 == 0) { // Cada 200ms
            printf("G_Roll: %.2f | Out: %d | Kp: %.3f\n", gyro_roll, m_out, pid_roll.kp);
        }
    }
}


// Función para leer la consola sin bloquear el vuelo


// ---- Fin Main -------




// --- IMPLEMENTACIÓN DE FUNCIONES AUXILIARES ---

void init_hardware() {
    // SPI MPU6500
    spi_init(MPU_SPI_INST, 20 * 1000 * 1000);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    sleep_ms(100);
    mpu_write(REG_PWR_MGMT_1, 0x01);
    mpu_write(REG_GYRO_CONFIG, 0x18);

    // PIO PPM (Usamos pio0, SM 0)
    uint offset_ppm = pio_add_program(pio0, &ppm_rx_program);
    pio_sm_config c_ppm = ppm_rx_program_get_default_config(offset_ppm);
    sm_config_set_in_pins(&c_ppm, PPM_PIN);
    sm_config_set_clkdiv(&c_ppm, 62.5f);
    pio_gpio_init(pio0, PPM_PIN);
    gpio_set_pulls(PPM_PIN, true, false);
    pio_sm_init(pio0, 0, offset_ppm, &c_ppm);
    pio_sm_set_enabled(pio0, 0, true);

    // PIO DShot (Usamos pio1 para no saturar pio0)
    uint offset_ds = pio_add_program(pio1, &dshot300_program);
    for(int i=0; i<4; i++) {
        motores[i].pio = pio1;
        motores[i].sm = i; // Usamos los 4 SMs de pio1
        dshot300_program_init(motores[i].pio, motores[i].sm, offset_ds, MOTOR_BASE_PIN + i);
    }
    
    // Calibración/Armado ESCs
    for (int i = 0; i < 100; i++) {
        send_dshot_all(0,0,0,0);
        sleep_ms(10);
    }
}

void process_ppm() {
    while (!pio_sm_is_rx_fifo_empty(pio0, 0)) {
        uint32_t val = pio_sm_get(pio0, 0);
        if (val > SYNC_THRESHOLD) {
            current_channel = 0;
        } else if (current_channel < NUM_CHANNELS) {
            if (val > 400 && val < 2400) raw_values[current_channel] = val;
            current_channel++;
        }
    }
}

float calcular_pid(pid_axis_t *pid, float setpoint, float lectura_actual) {
    float error = setpoint - lectura_actual;
    pid->integral += pid->ki * error;
    if (pid->integral > 400) pid->integral = 400;
    else if (pid->integral < -400) pid->integral = -400;
    float d_term = pid->kd * (error - pid->error_previo);
    pid->error_previo = error;
    return (pid->kp * error) + pid->integral + d_term;
}