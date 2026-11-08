#include "mpu6050.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

#define I2C_TIMEOUT_MS    50

#define GRAVITY_M_S2      9.80665f
#define ACC_LSB_PER_G     8192.0f   /* +-4 g */
#define GYRO_LSB_PER_DPS  65.5f     /* +-500 deg/s */

static esp_err_t write_reg(mpu6050_t *mpu, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(mpu->dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t read_regs(mpu6050_t *mpu, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(mpu->dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

esp_err_t mpu6050_init(mpu6050_t *mpu, i2c_master_bus_handle_t bus, uint8_t addr,
                       mpu6050_dlpf_t dlpf, uint32_t scl_hz)
{
    if (mpu == NULL || bus == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (mpu->dev == NULL)
    {
        i2c_device_config_t dev_conf = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = scl_hz,
        };
        esp_err_t ret = i2c_master_bus_add_device(bus, &dev_conf, &mpu->dev);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    uint8_t who = 0;
    esp_err_t ret = read_regs(mpu, REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK)
    {
        return ret;
    }
    /* El MPU6050 original responde 0x68; algunos clones 0x70, 0x72 o 0x98 */
    if (who != 0x68)
    {
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X (se esperaba 0x68). Puede ser un clon, se continua.", who);
    }

    /* Reset del chip y luego reloj del PLL del giroscopio X (mas estable
     * que el oscilador interno) */
    ret = write_reg(mpu, REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = write_reg(mpu, REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Con el DLPF activo la base es 1 kHz: 1000 / (1 + 4) = 200 Hz */
    ret = write_reg(mpu, REG_SMPLRT_DIV, 4);
    if (ret != ESP_OK) return ret;
    ret = write_reg(mpu, REG_CONFIG, (uint8_t)dlpf & 0x07);
    if (ret != ESP_OK) return ret;
    ret = write_reg(mpu, REG_GYRO_CONFIG, 0x08);    /* FS_SEL = 1: +-500 deg/s */
    if (ret != ESP_OK) return ret;
    ret = write_reg(mpu, REG_ACCEL_CONFIG, 0x08);   /* AFS_SEL = 1: +-4 g */
    if (ret != ESP_OK) return ret;

    mpu->acc_scale  = GRAVITY_M_S2 / ACC_LSB_PER_G;
    mpu->gyro_scale = ((float)M_PI / 180.0f) / GYRO_LSB_PER_DPS;

    vTaskDelay(pdMS_TO_TICKS(50));   /* que el filtro interno se asiente */
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_t *mpu, mpu6050_data_t *out)
{
    if (mpu == NULL || out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t b[14];
    esp_err_t ret = read_regs(mpu, REG_ACCEL_XOUT_H, b, sizeof(b));
    if (ret != ESP_OK)
    {
        return ret;
    }

    int16_t ax = (int16_t)((b[0] << 8) | b[1]);
    int16_t ay = (int16_t)((b[2] << 8) | b[3]);
    int16_t az = (int16_t)((b[4] << 8) | b[5]);
    int16_t t  = (int16_t)((b[6] << 8) | b[7]);
    int16_t gx = (int16_t)((b[8] << 8) | b[9]);
    int16_t gy = (int16_t)((b[10] << 8) | b[11]);
    int16_t gz = (int16_t)((b[12] << 8) | b[13]);

    out->ax = ax * mpu->acc_scale;
    out->ay = ay * mpu->acc_scale;
    out->az = az * mpu->acc_scale;
    out->gx = gx * mpu->gyro_scale;
    out->gy = gy * mpu->gyro_scale;
    out->gz = gz * mpu->gyro_scale;
    out->temp_c = t / 340.0f + 36.53f;
    return ESP_OK;
}
