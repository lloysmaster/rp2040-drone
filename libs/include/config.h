#ifndef CONFIG_H
#define CONFIG_H

#include "hardware/spi.h"

// --- ASIGNACIÓN DE PINES (Hardware Map) ---
#define PPM_PIN          2
#define MPU_SPI_INST     spi1
#define PIN_MISO         12
#define PIN_CS           13
#define PIN_SCK          10
#define PIN_MOSI         11
//se asignan +3 pines para los motores
#define MOTOR_BASE_PIN   16 

// --- PARÁMETROS DE VUELO ---
#define LOOP_FREQ_HZ     2000
#define TARGET_LOOP_US   (1000000 / LOOP_FREQ_HZ)
#define NUM_CHANNELS     8
#define SYNC_THRESHOLD   2700

// --- PARÁMETROS DE FILTRADO Y PID ---
#define FILTER_CUTOFF_HZ  80.0f
#define FILTER_DT        (1.0f / LOOP_FREQ_HZ)
#define FILTER_RC        (1.0f / (2.0f * 3.14159f * FILTER_CUTOFF_HZ))
#define FILTER_ALPHA     (FILTER_DT / (FILTER_RC + FILTER_DT))
#define PID_LIMIT_I      100.0f
// Puedes poner aquí tus Kp, Ki, Kd iniciales
#define ROLL_KP          0.0f
#define ROLL_KI          0.0f
#define ROLL_KD          0.0f

#define PITCH_KP          0.0f
#define PITCH_KI          0.0f
#define PITCH_KD          0.0f

#define YAW_KP          0.0f
#define YAW_KI          0.0f
#define YAW_KD          0.0f

// --- PROTOCOLO MOTORES (DShot) ---
#define DSHOT_MIN        48
#define DSHOT_MAX        2000
#define IDLE_THRESHOLD   50

// --- REGISTROS MPU6500 ---
#define REG_GYRO_CONFIG  0x1B
#define REG_PWR_MGMT_1   0x6B
#define REG_GYRO_XOUT_H  0x43
#define READ_BIT         0x80
// --- ajustes de control ---
#define STICK_SENSITIVITY 10.0f

// GYRO_CONFIG = 0x18 → FS_SEL=3 → ±2000°/s
// Cambiar GYRO_CONFIG requiere actualizar GYRO_SCALE
#define GYRO_SCALE  16.4f  // LSB / (°/s)

// === MODO SIMULACION ---
#define HIL_MODE true

#endif