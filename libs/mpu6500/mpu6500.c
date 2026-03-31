#include "mpu6500.h"
static inline void cs_select() {
    asm volatile("nop");
    gpio_put(PIN_CS, 0);
    asm volatile("nop");
}

static inline void cs_deselect() {
    asm volatile("nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop");
}

void mpu6500_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buf[] = {reg, data};
    cs_select();
    spi_write_blocking(MPU_SPI_INST, buf, 2);
    cs_deselect();
}

void mpu6500_init() {
    // Configuración de pines y SPI se asume hecha en init_hardware o aquí
    sleep_ms(100);
    mpu6500_write_reg(REG_PWR_MGMT_1, 0x01); // Clock source auto-select
    mpu6500_write_reg(REG_GYRO_CONFIG, 0x18); // 2000 dps full scale
}

void mpu6500_read_raw(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t reg = REG_GYRO_XOUT_H | 0x80; // READ_BIT
    uint8_t buffer[6];
    cs_select();
    spi_write_blocking(MPU_SPI_INST, &reg, 1);
    spi_read_blocking(MPU_SPI_INST, 0, buffer, 6);
    cs_deselect();

    *gx = (buffer[0] << 8) | buffer[1];
    *gy = (buffer[2] << 8) | buffer[3];
    *gz = (buffer[4] << 8) | buffer[5];
}

void mpu6500_calibrate(float *ox, float *oy, float *oz) {
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int muestras = 500;
    
    for (int i = 0; i < muestras; i++) {
        int16_t gx, gy, gz;
        mpu6500_read_raw(&gx, &gy, &gz);
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        sleep_us(1000);
    }
    *ox = (float)gx_sum / muestras;
    *oy = (float)gy_sum / muestras;
    *oz = (float)gz_sum / muestras;
}