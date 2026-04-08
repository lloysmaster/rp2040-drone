#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
// includes de libs
#include "config.h"
#include "mpu6500.h"
#include "radio.h"
#include "pid.h"
#include "dshot.h"
#include "telemetry.h"
#include "console_handler.h"
#include "mixer.h"
#include "filter.h"
// Archivos PIO generados
#include "pio_ppm_rx.pio.h"
#include "pio_dshot.pio.h"

// --- ESTRUCTURAS DE DATOS ---
pid_axis_t pid_roll, pid_pitch, pid_yaw;
ppm_rx_t receptor;
dshot_config_t esc_motores;
uint16_t motor_test_val = 0;
telemetry_data_t t_data;
ema_filter_t filter_roll, filter_pitch, filter_yaw;

auto_init_mutex(my_mutex);

void core1_entry();

float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
// --- PROTOTIPOS DE FUNCIONES ---
void init_hardware();
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
    mpu6500_write_reg(REG_PWR_MGMT_1, 0x01);
    mpu6500_write_reg(REG_GYRO_CONFIG, 0x18);

    radio_init(&receptor, pio0, 0, PPM_PIN);
    filter_init(&filter_roll, FILTER_ALPHA);
    filter_init(&filter_pitch, FILTER_ALPHA);
    filter_init(&filter_yaw, FILTER_ALPHA);

    // Inicializar con límites de anti-windup (ej: 100)
    pid_init(&pid_roll, ROLL_KP, ROLL_KI, ROLL_KD, PID_LIMIT_I);
    pid_init(&pid_pitch, PITCH_KP, PITCH_KI, PITCH_KD, PID_LIMIT_I);
    pid_init(&pid_yaw, YAW_KP, YAW_KI, YAW_KD, PID_LIMIT_I);

    // Configuración PIO DShot (pio1, 4 SMs)
    uint offset_ds = pio_add_program(pio1, &dshot300_program);
    dshot_init(&esc_motores, pio1, MOTOR_BASE_PIN);

    mpu6500_calibrate(&gyro_x_offset, &gyro_y_offset, &gyro_z_offset);
}
void core1_entry()
{
    while (true)
    {
        // 1. Actualizar Radio
        mutex_enter_blocking(&my_mutex);
        radio_update(&receptor);
        mutex_exit(&my_mutex);
        // 2. Manejar Consola (Ahora con los 3 ejes)
        mutex_enter_blocking(&my_mutex);
        // Agregamos &pid_yaw a los argumentos
        console_handle_input(&pid_roll, &pid_pitch, &pid_yaw, &motor_test_val);
        mutex_exit(&my_mutex);

        // 3. Telemetría (~25Hz)
        static uint32_t last_telemetry = 0;
        if (time_us_32() - last_telemetry > 40000)
        {
            telemetry_send(&t_data);
            last_telemetry = time_us_32();
        }

        sleep_us(100);
    }
}
// --- BUCLE PRINCIPAL (MAIN) ---

int main()
{
    init_hardware();
    multicore_launch_core1(core1_entry);
    int16_t gx, gy, gz;
    uint32_t next_loop_time = time_us_32();

    while (true)
    {
        // Control de tiempo del loop
        while (time_us_32() < next_loop_time)
            ;

        next_loop_time += TARGET_LOOP_US;

        // 1. Entradas (Consola y Radio)
        uint32_t chan_aileron = radio_get_channel(&receptor, 0);  // Roll
        uint32_t chan_elevator = radio_get_channel(&receptor, 1); // Pitch
        uint32_t chan_throttle = radio_get_channel(&receptor, 2); // Throttle
        uint32_t chan_rudder = radio_get_channel(&receptor, 3);   // Yaw

        // 2. Sensores y Filtrado
        mpu6500_read_raw(&gx, &gy, &gz);
        float roll_raw = ((float)gx - gyro_x_offset) / GYRO_SCALE;
        float pitch_raw = ((float)gy - gyro_y_offset) / GYRO_SCALE;
        float yaw_raw = ((float)gz - gyro_z_offset) / GYRO_SCALE;

        // Aplicar filtros
        float roll_f = filter_apply(&filter_roll, roll_raw);
        float pitch_f = filter_apply(&filter_pitch, pitch_raw);
        float yaw_f = filter_apply(&filter_yaw, yaw_raw);

        // 3. Procesamiento PID
        mutex_enter_blocking(&my_mutex);
        float setpoint_roll = ((float)chan_aileron - 1500) / STICK_SENSITIVITY;
        float setpoint_pitch = ((float)chan_elevator - 1500) / STICK_SENSITIVITY;
        float setpoint_yaw = ((float)chan_rudder - 1500) / STICK_SENSITIVITY; // Asumiendo CH4 es Yaw
        mutex_exit(&my_mutex);

        float corr_roll = pid_update(&pid_roll, setpoint_roll, roll_f);
        float corr_pitch = pid_update(&pid_pitch, setpoint_pitch, pitch_f);
        float corr_yaw = pid_update(&pid_yaw, setpoint_yaw, yaw_f);
        if (motor_test_val > 10000)
        {
            // Si el valor es especial, enviamos comando de Beacon
            uint16_t beacon_cmd = motor_test_val - 10000;

            // IMPORTANTE: Para que el ESC acepte un comando, el bit de telemetría
            // suele ser necesario o el comando debe repetirse.
            dshot_send_all(&esc_motores, beacon_cmd, beacon_cmd, beacon_cmd, beacon_cmd);

            // Opcional: El beacon suele sonar mejor si se envía en ráfagas o
            // simplemente se deja de enviar aceleración.
        }
        else
        {
            // 1. Obtenemos el valor real del acelerador del radio (mapeado de 1000-2000 a 0-1000 aprox, o según tu mixer)
            int32_t throttle_val = (int32_t)chan_throttle - 1000;
            if (throttle_val < 50)
                throttle_val = 0;

            // Sumamos el valor de consola (test) y limitamos a 1000
            uint16_t total_throttle = (uint16_t)(throttle_val + motor_test_val);
            if (total_throttle > 1000)
                total_throttle = 1000;

            motor_output_t m_out = mixer_compute(total_throttle, corr_roll, corr_pitch, corr_yaw);
            dshot_send_all(&esc_motores, m_out.m1, m_out.m2, m_out.m3, m_out.m4);

            // 6. Telemetría actualizada
            static int count = 0;
            if (count++ % 20 == 0)
            { // Frecuencia de ~25Hz a 500Hz de loop
                t_data.roll = roll_f;
                t_data.pitch = pitch_f;
                t_data.yaw = yaw_f;
                t_data.motors[0] = m_out.m1;
                t_data.motors[1] = m_out.m2;
                t_data.motors[2] = m_out.m3;
                t_data.motors[3] = m_out.m4;
                t_data.throttle = total_throttle;

                telemetry_send(&t_data);
            }
        }
    }
    // --- CORE 1: COMUNICACIONES Y TAREAS LENTAS ---
}