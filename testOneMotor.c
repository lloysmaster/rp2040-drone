#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// Archivos PIO generados
#include "pio_ppm_rx.pio.h"
#include "pio_dshot.pio.h"

// --- CONFIGURACIÓN DE PINES ---
#define PPM_PIN 2
#define MPU_SPI_INST spi1
#define PIN_MISO 12
#define PIN_CS 13
#define PIN_SCK 10
#define PIN_MOSI 11
#define MOTOR_BASE_PIN 16 // Motores en pines 16, 17, 18, 19

// --- CONFIGURACIÓN MPU6500 (REGISTROS) ---
#define REG_GYRO_CONFIG 0x1B
#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_XOUT_H 0x43
#define READ_BIT 0x80

// --- PARÁMETROS DE RECEPCIÓN PPM ---
#define NUM_CHANNELS 8
#define SYNC_THRESHOLD 2700

// --- ESTRUCTURAS DE DATOS ---
typedef struct
{
    float kp, ki, kd;
    float error_previo;
    float integral;
} pid_axis_t;

typedef struct
{
    PIO pio;
    uint sm;
} motor_t;

// --- VARIABLES GLOBALES ---
uint32_t raw_values[NUM_CHANNELS];
int current_channel = 0;
uint16_t motor_test_val = 0;

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5

// Filtros y offsets
float roll_filtrado = 0;
float pitch_filtrado = 0;
float alpha = 0.2f; // Factor de filtro complementario/EMA
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

// Instancias de PID
pid_axis_t pid_roll = {.kp = 0.00f, .ki = 0.000f, .kd = 0.00f};
pid_axis_t pid_pitch = {.kp = 0.00f, .ki = 0.000f, .kd = 0.00f};
pid_axis_t pid_yaw = {.kp = 0.00f, .ki = 0.000f, .kd = 0.00f};

// Instancias de Motores
motor_t motores[4];

// --- PROTOTIPOS DE FUNCIONES ---
void init_hardware();
void process_ppm();
void check_console_input();
void calibrar_sensores();
float calcular_pid(pid_axis_t *pid, float setpoint, float lectura_actual);
void send_dshot_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
uint16_t dshot_prepare_packet(uint16_t value);

// --- FUNCIONES MPU6500 (SPI) ---
static inline void cs_select() { gpio_put(PIN_CS, 0); }
static inline void cs_deselect() { gpio_put(PIN_CS, 1); }

void mpu_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[] = {reg, data};
    cs_select();
    spi_write_blocking(MPU_SPI_INST, buf, 2);
    cs_deselect();
}

void mpu_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
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

// --- LÓGICA DE CONTROL Y VUELO ---

void calibrar_sensores()
{
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int muestras = 500;
    printf("Calibrando... NO MUEVAS EL DRON\n");

    for (int i = 0; i < muestras; i++)
    {
        int16_t gx, gy, gz;
        mpu_read_gyro(&gx, &gy, &gz);
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        sleep_us(1000);
    }
    gyro_x_offset = (float)gx_sum / muestras;
    gyro_y_offset = (float)gy_sum / muestras;
    gyro_z_offset = (float)gz_sum / muestras;
    printf("Calibracion lista. Offsets: X:%.2f Y:%.2f\n", gyro_x_offset, gyro_y_offset);
}

float calcular_pid(pid_axis_t *pid, float setpoint, float lectura_actual)
{
    float error = setpoint - lectura_actual;

    // Término Integral con anti-windup
    pid->integral += pid->ki * error;
    if (pid->integral > 100)
        pid->integral = 100;
    else if (pid->integral < -100)
        pid->integral = -100;

    // Término Derivativo
    float d_term = pid->kd * (error - pid->error_previo);
    pid->error_previo = error;

    return (pid->kp * error) + pid->integral + d_term;
}

// --- LÓGICA DSHOT ---

uint16_t dshot_prepare_packet(uint16_t value)
{
    if (value == 0)
        return 0;
    uint16_t packet = (value << 1);
    uint16_t checksum = 0;
    uint16_t temp_packet = packet;

    for (int i = 0; i < 3; i++)
    {
        checksum ^= (temp_packet & 0x0F);
        temp_packet >>= 4;
    }
    return (packet << 4) | (checksum & 0x0F);
}

void send_dshot_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    uint16_t vals[4] = {m1, m2, m3, m4};
    for (int i = 0; i < 4; i++)
    {
        uint16_t packet = dshot_prepare_packet(vals[i]);
        pio_sm_put_blocking(motores[i].pio, motores[i].sm, packet << 16);
    }
}

// --- COMUNICACIÓN Y ENTRADA ---

void check_console_input()
{
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT)
    {
        static char buffer[32];
        static int idx = 0;

        if (c == '\n' || c == '\r')
        {
            buffer[idx] = '\0';
            if (idx > 0)
            {
                char type = buffer[0];
                float val = atof(&buffer[1]);

                if (type == 'M' || type == 'm')
                {
                    motor_test_val = (uint16_t)val;
                    printf(">> Motor Test Set: %d\n", motor_test_val);
                }
                else if (type == 'P' || type == 'p')
                {
                    pid_roll.kp = pid_pitch.kp = val;
                    printf(">> Kp (P) actualizado a: %.4f\n", val);
                }
                else if (type == 'I' || type == 'i')
                {
                    pid_roll.ki = pid_pitch.ki = val;
                    pid_roll.integral = pid_pitch.integral = 0;
                    printf(">> Ki (I) actualizado a: %.4f (Integral reseteada)\n", val);
                }
                else if (type == 'D' || type == 'd')
                {
                    pid_roll.kd = pid_pitch.kd = val;
                    printf(">> Kd (D) actualizado a: %.4f\n", val);
                }
                else if (type == 'S' || type == 's')
                {
                    printf("\n--- PID STATUS ---\nP: %.4f | I: %.4f | D: %.4f\n------------------\n",
                           pid_roll.kp, pid_roll.ki, pid_roll.kd);
                }
            }
            idx = 0;
        }
        else if (idx < 31)
        {
            buffer[idx++] = (char)c;
        }
    }
}

void process_ppm()
{
    while (!pio_sm_is_rx_fifo_empty(pio0, 0))
    {
        uint32_t val = pio_sm_get(pio0, 0);
        if (val > SYNC_THRESHOLD)
        {
            current_channel = 0;
        }
        else if (current_channel < NUM_CHANNELS)
        {
            if (val > 400 && val < 2400)
                raw_values[current_channel] = val;
            current_channel++;
        }
    }
}

// --- INICIALIZACIÓN DE HARDWARE ---

void init_hardware()
{
    stdio_init_all();

    // Configuración SPI para MPU6500
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

    // Configuración PIO PPM (pio0, SM 0)
    uint offset_ppm = pio_add_program(pio0, &ppm_rx_program);
    pio_sm_config c_ppm = ppm_rx_program_get_default_config(offset_ppm);
    sm_config_set_in_pins(&c_ppm, PPM_PIN);
    sm_config_set_clkdiv(&c_ppm, 62.5f);
    pio_gpio_init(pio0, PPM_PIN);
    gpio_set_pulls(PPM_PIN, true, false);
    pio_sm_init(pio0, 0, offset_ppm, &c_ppm);
    pio_sm_set_enabled(pio0, 0, true);

    // Configuración PIO DShot (pio1, 4 SMs)
    uint offset_ds = pio_add_program(pio1, &dshot300_program);
    for (int i = 0; i < 4; i++)
    {
        motores[i].pio = pio1;
        motores[i].sm = i;
        dshot300_program_init(motores[i].pio, motores[i].sm, offset_ds, MOTOR_BASE_PIN + i);
    }

    // Inicialización ESCs (Armado)
    for (int i = 0; i < 100; i++)
    {
        send_dshot_all(0, 0, 0, 0);
        sleep_ms(10);
    }

    calibrar_sensores();
}


// --- BUCLE PRINCIPAL (MAIN) ---

int main()
{
    init_hardware();
    printf("Modo Test: Usa 'Mxxx' para motor (48-2000) o 'Pxxx' para Kp\n");

    int16_t gx, gy, gz;
    uint32_t target_loop_time_us = 2000; // Frecuencia de 500Hz
    uint32_t next_loop_time = time_us_32();

    while (true)
    {
        // Control de tiempo del loop
        while (time_us_32() < next_loop_time)
            ;
        next_loop_time += target_loop_time_us;

        // 1. Entradas (Consola y Radio)
        check_console_input();
        process_ppm();
        float throttle_input = raw_values[2];

        // 2. Sensores y Filtrado
        mpu_read_gyro(&gx, &gy, &gz);
        float roll_raw = ((float)gx - gyro_x_offset) / 16.4f;
        float pitch_raw = ((float)gy - gyro_y_offset) / 16.4f;
        float yaw_raw = ((float)gz - gyro_z_offset) / 16.4f;

        roll_filtrado = (roll_filtrado * (1.0f - alpha)) + (roll_raw * alpha);
        pitch_filtrado = (pitch_filtrado * (1.0f - alpha)) + (pitch_raw * alpha);

        // 3. Procesamiento PID
        float setpoint_roll = ((float)raw_values[0] - 1500) / 10.0f;
        float setpoint_pitch = ((float)raw_values[1] - 1500) / 10.0f;
        float setpoint_yaw = ((float)raw_values[3] - 1500) / 10.0f; // Asumiendo CH4 es Yaw

        // Si quieres filtrar el yaw también (opcional pero recomendado):
        static float yaw_filtrado = 0;
        yaw_filtrado = (yaw_filtrado * (1.0f - alpha)) + (yaw_raw * alpha);

        float corr_roll = calcular_pid(&pid_roll, setpoint_roll, roll_filtrado);
        float corr_pitch = calcular_pid(&pid_pitch, setpoint_pitch, pitch_filtrado);
        float corr_yaw = calcular_pid(&pid_yaw, setpoint_yaw, yaw_raw);

        // 4. Mezcla de Motores (Mixer Cuadricóptero en X)
        /*
           Estructura típica:
           Motor 1 (Front R): Throttle - Roll - Pitch + Yaw
           Motor 2 (Front L): Throttle + Roll - Pitch - Yaw
           Motor 3 (Rear R):  Throttle - Roll + Pitch - Yaw
           Motor 4 (Rear L):  Throttle + Roll + Pitch + Yaw
        */

        uint16_t throttle = (motor_test_val > 0) ? motor_test_val : (uint16_t)raw_values[2];

        int16_t m1_calc = throttle - corr_roll - corr_pitch + corr_yaw;
        int16_t m2_calc = throttle + corr_roll - corr_pitch - corr_yaw;
        int16_t m3_calc = throttle - corr_roll + corr_pitch - corr_yaw;
        int16_t m4_calc = throttle + corr_roll + corr_pitch + corr_yaw;

        // 5. Salida DShot y Seguridad para los 4 motores
        uint16_t m_out[4] = {0, 0, 0, 0};
        int16_t *calcs[4] = {&m1_calc, &m2_calc, &m3_calc, &m4_calc};

        for (int i = 0; i < 4; i++)
        {
            if (throttle < 48)
            {
                m_out[i] = 0; // Motores apagados si el throttle es bajo
            }
            else
            {
                if (*calcs[i] < 48)
                    m_out[i] = 48;
                else if (*calcs[i] > 2000)
                    m_out[i] = 2000;
                else
                    m_out[i] = (uint16_t)*calcs[i];
            }
        }

        // Enviamos a los 4 aunque solo uno esté conectado físicamente
        send_dshot_all(m_out[0], m_out[1], m_out[2], m_out[3]);

        // 6. Telemetría actualizada
        static int count = 0;
        if (count++ % 50 == 0) // Aumentamos un poco la frecuencia (cada 100ms a 500Hz)
        {
            printf("M:[%4d %4d %4d %4d] | R:%6.2f | P:%6.2f | Y:%6.2f | Thr:%4d\n",
                   m_out[0], m_out[1], m_out[2], m_out[3],
                   roll_filtrado,
                   pitch_filtrado,
                   yaw_raw, // O yaw_filtrado si lo usaste
                   throttle);
        }
    }
}