#ifndef MPU6050_H
#define MPU6050_H

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR_AD0_LOW   0x68
#define MPU6050_ADDR_AD0_HIGH  0x69

/* Filtro pasa-bajos digital interno (registro CONFIG, DLPF_CFG).
 * Ancho de banda aproximado acelerometro / giroscopio. */
typedef enum
{
    MPU6050_DLPF_260HZ = 0,   /* 260 Hz / 256 Hz (sin filtro) */
    MPU6050_DLPF_184HZ = 1,   /* 184 Hz / 188 Hz */
    MPU6050_DLPF_94HZ  = 2,   /*  94 Hz /  98 Hz */
    MPU6050_DLPF_44HZ  = 3,   /*  44 Hz /  42 Hz */
    MPU6050_DLPF_21HZ  = 4,   /*  21 Hz /  20 Hz */
    MPU6050_DLPF_10HZ  = 5,   /*  10 Hz /  10 Hz */
    MPU6050_DLPF_5HZ   = 6,   /*   5 Hz /   5 Hz */
} mpu6050_dlpf_t;

typedef struct
{
    i2c_master_dev_handle_t dev;
    float acc_scale;    /* (m/s^2) por LSB */
    float gyro_scale;   /* (rad/s) por LSB */
} mpu6050_t;

typedef struct
{
    float ax, ay, az;   /* m/s^2, incluye la gravedad */
    float gx, gy, gz;   /* rad/s */
    float temp_c;       /* grados Celsius */
} mpu6050_data_t;

/**
 * @brief Agrega el MPU6050 al bus I2C, lo despierta y lo configura:
 *        giroscopio +-500 deg/s, acelerometro +-4 g, muestreo 200 Hz
 *        y el filtro pasa-bajos interno indicado.
 *
 * @param mpu     Estructura del sensor (se llena aqui).
 * @param bus     Bus I2C maestro ya creado.
 * @param addr    MPU6050_ADDR_AD0_LOW (0x68) o MPU6050_ADDR_AD0_HIGH (0x69).
 * @param dlpf    Ancho de banda del filtro pasa-bajos interno.
 * @param scl_hz  Frecuencia del I2C (hasta 400000).
 */
esp_err_t mpu6050_init(mpu6050_t *mpu, i2c_master_bus_handle_t bus, uint8_t addr,
                       mpu6050_dlpf_t dlpf, uint32_t scl_hz);

/**
 * @brief Lee acelerometro, temperatura y giroscopio en una sola
 *        transaccion I2C (14 bytes desde ACCEL_XOUT_H).
 */
esp_err_t mpu6050_read(mpu6050_t *mpu, mpu6050_data_t *out);

#ifdef __cplusplus
}
#endif

#endif
