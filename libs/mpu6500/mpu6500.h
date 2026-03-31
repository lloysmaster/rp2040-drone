#ifndef mpu6500_H
#define mpu6500_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "../include/config.h"

// Estructura para agrupar los datos procesados
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} mpu_data_t;

// Prototipos
void mpu6500_init();
void mpu6500_write_reg(uint8_t reg, uint8_t data);
void mpu6500_read_raw(int16_t *gx, int16_t *gy, int16_t *gz);
void mpu6500_calibrate(float *ox, float *oy, float *oz);

#endif