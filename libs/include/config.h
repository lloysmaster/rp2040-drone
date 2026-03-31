#ifndef CONFIG_H
#define CONFIG_H

#include "hardware/spi.h"

// --- CONFIGURACIÓN DE PINES ---
#define PPM_PIN         2
#define MPU_SPI_INST    spi1
#define PIN_MISO        12
#define PIN_CS          13
#define PIN_SCK         10
#define PIN_MOSI        11
#define MOTOR_BASE_PIN  16 // Motores en 16, 17, 18, 19

// --- PARÁMETROS DE HARDWARE ---
#define NUM_CHANNELS    8
#define SYNC_THRESHOLD  2700
#define LOOP_FREQ_HZ    500
#define TARGET_LOOP_US  (1000000 / LOOP_FREQ_HZ)

// --- REGISTROS MPU6500 ---
#define REG_GYRO_CONFIG 0x1B
#define REG_PWR_MGMT_1  0x6B
#define REG_GYRO_XOUT_H 0x43
#define READ_BIT        0x80

// --- CONSTANTES DSHOT ---
#define DSHOT_MIN       48
#define DSHOT_MAX       2000

#endif