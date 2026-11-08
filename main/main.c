/*
 * ============================================================
 * Guante de tele-operacion (Micro 1)
 *
 * MPU6050 en la mano del operador (orientacion con un filtro de Mahony
 * en el ESP32). Dos salidas en paralelo:
 *  - ESP-NOW -> ESP32 del carro (50 Hz): orientacion y movimiento de
 *    la mano para que el carro se mueva directamente, sin pasar por el
 *    PC (menor latencia y sigue funcionando si se cae el PC).
 *  - micro-ROS (WiFi) -> PC: los mismos datos para los filtros de
 *    Kalman, la visualizacion y la estrategia 1 (plano).
 *
 * Armado del mando directo (ESP-NOW): boton BOOT del ESP32
 *   pulsacion corta  -> armar / desarmar (LED encendido = armado)
 *   mantener 1 s     -> fijar la postura neutra actual (LED parpadea)
 * o desde el PC con glove/arm. Se arranca siempre desarmado.
 *
 * Topicos publicados (best effort, GLOVE_PUBLISH_HZ):
 *   glove/imu/raw       sensor_msgs/Imu  acelerometro (con gravedad) y
 *                       giroscopio sin filtrar. Para el EKF/UKF del PC.
 *   glove/imu/filtered  sensor_msgs/Imu  orientacion (quaternion del
 *                       filtro de Mahony), giroscopio y aceleracion
 *                       lineal (gravedad restada con esa orientacion)
 *                       con pasa-bajos EMA.
 *   glove/rpy           geometry_msgs/Vector3  x = roll, y = pitch,
 *                       z = yaw (rad), relativos a la postura neutra.
 *
 *   glove/armed         std_msgs/Bool  estado del mando directo
 *
 * Topicos suscritos:
 *   glove/zero          std_msgs/Empty  toma la postura actual de la
 *                       mano como neutra (roll = pitch = yaw = 0).
 *   glove/arm           std_msgs/Bool   arma / desarma el mando directo
 *
 * Todos los mensajes Imu llevan header.stamp sincronizado con el
 * reloj del agente, para que el filtro de Kalman del PC use el dt
 * real entre muestras.
 * ============================================================
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "mpu6050.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/bool.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/discovery.h>
#include "uros_network_interfaces.h"
#include "espnow_link.h"

const char *TAG = "Guante";

/* Ver main.c del carro: IDF compila con -Werror y rcl/rclc marcan
 * warn_unused_result. */
#define RCRETCHECK(fn)                                                           \
    do {                                                                         \
        rcl_ret_t rc_ = (fn);                                                    \
        if (rc_ != RCL_RET_OK) {                                                 \
            ESP_LOGE(TAG, "rcl error %d en linea %d: %s", (int)rc_, __LINE__, #fn); \
            return false;                                                        \
        }                                                                        \
    } while (0)

#define RCSOFTCHECK(fn)                                                          \
    do {                                                                         \
        rcl_ret_t rc_ = (fn);                                                    \
        (void)rc_;                                                               \
    } while (0)

/*
 * ============================================================
 * MPU6050 (I2C)
 *   VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22,
 *   AD0 -> GND (0x68) o 3V3 (0x69): la direccion se detecta sola.
 * ============================================================
 */
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_SCL_IO   22
#define I2C_FREQ_HZ         400000
#define MPU_DLPF            MPU6050_DLPF_44HZ   /* quita la vibracion sin retrasar el movimiento de la mano */

/*
 * ============================================================
 * Muestreo, fusion y filtrado de bajo nivel
 *  El MPU6050 no tiene fusion interna (el BNO055 si la tenia): la
 *  orientacion se calcula aqui con un filtro de Mahony sobre el
 *  quaternion. El giroscopio integra el movimiento rapido y el
 *  acelerometro (direccion de la gravedad) corrige la deriva de roll y
 *  pitch. Sin magnetometro, el yaw es relativo y deriva lentamente.
 * ============================================================
 */
#define IMU_PERIOD_MS       10      /* 100 Hz */
#define GLOVE_PUBLISH_HZ    50
#define GYRO_FILTER_ALPHA   0.2f    /* EMA a 100 Hz: corte ~3.5 Hz (movimiento de la mano < 3 Hz) */
#define ACC_FILTER_ALPHA    0.2f
#define MAHONY_KP           1.5f    /* correccion con el acelerometro (1/s): mas alto = sigue antes a la gravedad */
#define MAHONY_KP_START     8.0f    /* los primeros segundos: converge rapido a la postura inicial */
#define MAHONY_START_S      2.0f
#define ACC_TRUST_BAND      0.15f   /* el acelerometro corrige solo si |a| = g +-15 % (mano sin acelerar) */
#define GYRO_BIAS_SAMPLES   100     /* 1 s a 100 Hz, al arrancar (guante quieto) */
#define GYRO_STILL_RAD_S    0.05f   /* por debajo, la mano esta quieta: se afina el sesgo */
#define GYRO_BIAS_ALPHA     0.002f  /* EMA del sesgo con la mano quieta (~5 s) */
#define GRAVITY_M_S2        9.80665f

/*
 * ============================================================
 * ESP-NOW, boton y LED
 * ============================================================
 */
#define ESPNOW_PERIOD_MS    20      /* 50 Hz hacia el carro */
#define ARM_BUTTON_GPIO     GPIO_NUM_0    /* boton BOOT de la placa (activo en bajo) */
#define STATUS_LED_GPIO     GPIO_NUM_2    /* LED de la placa; -1 si no hay */
#define LONG_PRESS_MS       1000
#define GLOVE_DEBUG_LOG     1       /* 1: lo que se envia al carro, en el monitor cada 1 s */
#define GLOVE_TELEMETRY     1       /* 1: linea CSV "G,..." a 50 Hz por el monitor (analisis) */

typedef struct
{
    float raw_ax, raw_ay, raw_az;      /* m/s^2, acelerometro (incluye gravedad) */
    float raw_gx, raw_gy, raw_gz;      /* rad/s, giroscopio (sin sesgo) */
    float f_lin_ax, f_lin_ay, f_lin_az;/* m/s^2, aceleracion lineal filtrada */
    float f_gx, f_gy, f_gz;            /* rad/s, giroscopio filtrado */
    float qx, qy, qz, qw;              /* quaternion del filtro de Mahony */
    float roll, pitch, yaw;            /* rad, desde el quaternion */
    bool fusion_ok;                    /* la fusion ya entrega orientacion */
} glove_data_t;

static glove_data_t g_data = {0};
static portMUX_TYPE g_data_lock = portMUX_INITIALIZER_UNLOCKED;

/* Postura neutra (glove/zero) */
static volatile bool g_zero_req = true;     /* se fija sola al arrancar */
static float g_roll0 = 0.0f, g_pitch0 = 0.0f, g_yaw0 = 0.0f;

/* Mando directo por ESP-NOW */
static volatile bool g_armed = false;
static volatile bool g_espnow_ok = false;

static mpu6050_t imu = {0};
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static uint8_t s_mpu_addr = MPU6050_ADDR_AD0_LOW;
static volatile uint32_t g_i2c_fail = 0;    /* lecturas I2C fallidas */
static volatile float g_qnorm = 0.0f;       /* |q|: ~1 si la orientacion es valida */
static volatile bool g_bias_ok = false;     /* sesgo del giroscopio calibrado */

static float wrap_pi(float a)
{
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* ============================================================
 * MPU6050: bus I2C, deteccion y configuracion
 * ============================================================ */
static void mpu_setup(void)
{
    i2c_master_bus_config_t conf = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &s_i2c_bus));

    /* Escaneo: que dispositivos responden y en que direccion esta el
     * MPU6050 (0x68 con AD0 a GND, 0x69 con AD0 a 3V3) */
    for (int intento = 1; ; intento++)
    {
        char found[96] = "";
        int n = 0;
        for (uint16_t a = 0x08; a < 0x78; a++)
        {
            if (i2c_master_probe(s_i2c_bus, a, 20) == ESP_OK)
            {
                n += snprintf(found + n, sizeof(found) - n, " 0x%02X", a);
                if (n >= (int)sizeof(found) - 6) break;
            }
        }
        ESP_LOGI(TAG, "Escaneo I2C (SDA=GPIO%d, SCL=GPIO%d):%s", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO,
                 found[0] ? found : " ningun dispositivo");
        if (i2c_master_probe(s_i2c_bus, MPU6050_ADDR_AD0_LOW, 20) == ESP_OK)  { s_mpu_addr = MPU6050_ADDR_AD0_LOW;  break; }
        if (i2c_master_probe(s_i2c_bus, MPU6050_ADDR_AD0_HIGH, 20) == ESP_OK) { s_mpu_addr = MPU6050_ADDR_AD0_HIGH; break; }
        ESP_LOGE(TAG, "MPU6050 no responde (intento %d). Revisa: VCC->3V3, GND, SDA->GPIO%d, SCL->GPIO%d, AD0. Reintentando...",
                 intento, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    for (int intento = 1; ; intento++)
    {
        esp_err_t r = mpu6050_init(&imu, s_i2c_bus, s_mpu_addr, MPU_DLPF, I2C_FREQ_HZ);
        if (r == ESP_OK)
        {
            break;
        }
        ESP_LOGE(TAG, "MPU6050 en 0x%02X no se pudo configurar (intento %d: %s). Reintentando...",
                 s_mpu_addr, intento, esp_err_to_name(r));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "MPU6050 configurado en 0x%02X (+-500 deg/s, +-4 g, DLPF 44 Hz). Deja el guante quieto 1 s: calibrando giroscopio...",
             s_mpu_addr);
}

/* ============================================================
 * Filtro de Mahony (solo acelerometro + giroscopio)
 *  q' = 0.5 q (x) (w + Kp e), con e = a_medida x g_estimada: el error
 *  entre la gravedad que mide el acelerometro y la que predice el
 *  quaternion empuja la orientacion hacia la correcta.
 * ============================================================ */
static float s_q0 = 1.0f, s_q1 = 0.0f, s_q2 = 0.0f, s_q3 = 0.0f;   /* w, x, y, z */

static void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float kp, float dt)
{
    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an > 1e-3f && fabsf(an - GRAVITY_M_S2) < ACC_TRUST_BAND * GRAVITY_M_S2)
    {
        ax /= an; ay /= an; az /= an;
        /* gravedad estimada en el marco del sensor (tercera fila de R) */
        float vx = 2.0f * (s_q1 * s_q3 - s_q0 * s_q2);
        float vy = 2.0f * (s_q0 * s_q1 + s_q2 * s_q3);
        float vz = s_q0 * s_q0 - s_q1 * s_q1 - s_q2 * s_q2 + s_q3 * s_q3;
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;
        gx += kp * ex;
        gy += kp * ey;
        gz += kp * ez;
    }
    float hdt = 0.5f * dt;
    float q0 = s_q0, q1 = s_q1, q2 = s_q2, q3 = s_q3;
    s_q0 += (-q1 * gx - q2 * gy - q3 * gz) * hdt;
    s_q1 += ( q0 * gx + q2 * gz - q3 * gy) * hdt;
    s_q2 += ( q0 * gy - q1 * gz + q3 * gx) * hdt;
    s_q3 += ( q0 * gz + q1 * gy - q2 * gx) * hdt;
    float qn = sqrtf(s_q0 * s_q0 + s_q1 * s_q1 + s_q2 * s_q2 + s_q3 * s_q3);
    if (qn > 1e-6f)
    {
        s_q0 /= qn; s_q1 /= qn; s_q2 /= qn; s_q3 /= qn;
    }
}

/* Postura inicial directamente desde el acelerometro (roll, pitch; yaw 0) */
static void mahony_init_from_acc(float ax, float ay, float az)
{
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    s_q0 = cr * cp; s_q1 = sr * cp; s_q2 = cr * sp; s_q3 = -sr * sp;
}

/* ============================================================
 * Tarea de IMU (100 Hz)
 *  1. Al arrancar promedia el giroscopio 1 s (guante quieto) para
 *     estimar su sesgo; despues lo sigue afinando cuando la mano esta
 *     quieta.
 *  2. Filtro de Mahony -> quaternion -> roll, pitch, yaw.
 *  3. Aceleracion lineal = acelerometro - gravedad estimada.
 *  4. Pasa-bajos EMA al giroscopio y a la aceleracion lineal (temblor de
 *     la mano y ruido de alta frecuencia).
 * ============================================================ */
static void set_armed(bool on, const char *who);

void imu_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t fail_count = 0;
    int fail_streak = 0;
    int bias_n = 0;
    float bias_acc[3] = {0}, bias[3] = {0};
    bool first_out = true, q_init = false;
    int64_t t_fusion0 = 0, last_us = esp_timer_get_time();
    mpu6050_data_t d = {0};

    while (1)
    {
        esp_err_t r = mpu6050_read(&imu, &d);
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt > 0.05f) dt = 0.05f;

        if (r != ESP_OK)
        {
            g_i2c_fail = ++fail_count;
            fail_streak++;
            if (fail_count % 100 == 1)
            {
                ESP_LOGW(TAG, "MPU6050: fallo de lectura I2C (%lu en total)", (unsigned long)fail_count);
            }
            /* 0.5 s sin lecturas: el sensor se colgo o perdio alimentacion */
            if (fail_streak == 50 || (fail_streak > 50 && fail_streak % 100 == 0))
            {
                set_armed(false, "MPU6050 perdido");
                ESP_LOGW(TAG, "MPU6050 sin lecturas %d ms: reconfigurando (revisa VCC/GND/SDA/SCL)", fail_streak * IMU_PERIOD_MS);
                i2c_master_bus_reset(s_i2c_bus);
                if (mpu6050_init(&imu, s_i2c_bus, s_mpu_addr, MPU_DLPF, I2C_FREQ_HZ) == ESP_OK)
                {
                    ESP_LOGI(TAG, "MPU6050 recuperado. Vuelve a armar con BOOT.");
                }
                last_wake = xTaskGetTickCount();
            }
        }
        else
        {
            fail_streak = 0;

            /* 1. Sesgo del giroscopio */
            if (bias_n < GYRO_BIAS_SAMPLES)
            {
                bias_acc[0] += d.gx; bias_acc[1] += d.gy; bias_acc[2] += d.gz;
                if (++bias_n == GYRO_BIAS_SAMPLES)
                {
                    for (int i = 0; i < 3; i++) bias[i] = bias_acc[i] / GYRO_BIAS_SAMPLES;
                    mahony_init_from_acc(d.ax, d.ay, d.az);
                    q_init = true;
                    t_fusion0 = now_us;
                    g_bias_ok = true;
                    ESP_LOGI(TAG, "Giroscopio calibrado: sesgo=(%.4f, %.4f, %.4f) rad/s", bias[0], bias[1], bias[2]);
                }
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
                continue;
            }
            float gx = d.gx - bias[0], gy = d.gy - bias[1], gz = d.gz - bias[2];
            float an = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
            if (fabsf(gx) < GYRO_STILL_RAD_S && fabsf(gy) < GYRO_STILL_RAD_S && fabsf(gz) < GYRO_STILL_RAD_S &&
                fabsf(an - GRAVITY_M_S2) < 0.05f * GRAVITY_M_S2)
            {
                bias[0] += GYRO_BIAS_ALPHA * (d.gx - bias[0]);
                bias[1] += GYRO_BIAS_ALPHA * (d.gy - bias[1]);
                bias[2] += GYRO_BIAS_ALPHA * (d.gz - bias[2]);
            }

            /* 2. Fusion */
            float kp = ((now_us - t_fusion0) * 1e-6f < MAHONY_START_S) ? MAHONY_KP_START : MAHONY_KP;
            mahony_update(gx, gy, gz, d.ax, d.ay, d.az, kp, dt);

            /* 3. Aceleracion lineal (sin gravedad) en el marco del sensor */
            float vx = 2.0f * (s_q1 * s_q3 - s_q0 * s_q2);
            float vy = 2.0f * (s_q0 * s_q1 + s_q2 * s_q3);
            float vz = s_q0 * s_q0 - s_q1 * s_q1 - s_q2 * s_q2 + s_q3 * s_q3;
            float lax = d.ax - GRAVITY_M_S2 * vx;
            float lay = d.ay - GRAVITY_M_S2 * vy;
            float laz = d.az - GRAVITY_M_S2 * vz;

            float qw = s_q0, qx = s_q1, qy = s_q2, qz = s_q3;
            /* Roll como elevacion del eje y de la mano: asin(R[2][1]). Para
             * inclinaciones pequenas es igual al roll de Euler, pero no tiene
             * gimbal lock (con el de Euler, pasando ~85 deg de pitch el roll
             * saltaba de 0 a -167 deg y el carro giraba a fondo). */
            float roll  = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f * (qw * qx + qy * qz))));
            float pitch = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f * (qw * qy - qz * qx))));
            float yaw   = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
            g_qnorm = 1.0f;

            /* La postura neutra se fija cuando la fusion ya convergio */
            bool converged = q_init && (now_us - t_fusion0) * 1e-6f > MAHONY_START_S;
            if (g_zero_req && converged)
            {
                g_roll0 = roll;
                g_pitch0 = pitch;
                g_yaw0 = yaw;
                g_zero_req = false;
                ESP_LOGI(TAG, "Postura neutra fijada: roll=%.1f pitch=%.1f yaw=%.1f deg",
                         roll * 180.0f / (float)M_PI, pitch * 180.0f / (float)M_PI, yaw * 180.0f / (float)M_PI);
            }

            /* 4. Salidas */
            taskENTER_CRITICAL(&g_data_lock);
            g_data.raw_ax = d.ax;
            g_data.raw_ay = d.ay;
            g_data.raw_az = d.az;
            g_data.raw_gx = gx;
            g_data.raw_gy = gy;
            g_data.raw_gz = gz;
            if (first_out)
            {
                g_data.f_gx = gx; g_data.f_gy = gy; g_data.f_gz = gz;
                g_data.f_lin_ax = lax; g_data.f_lin_ay = lay; g_data.f_lin_az = laz;
                first_out = false;
            }
            else
            {
                g_data.f_gx += GYRO_FILTER_ALPHA * (gx - g_data.f_gx);
                g_data.f_gy += GYRO_FILTER_ALPHA * (gy - g_data.f_gy);
                g_data.f_gz += GYRO_FILTER_ALPHA * (gz - g_data.f_gz);
                g_data.f_lin_ax += ACC_FILTER_ALPHA * (lax - g_data.f_lin_ax);
                g_data.f_lin_ay += ACC_FILTER_ALPHA * (lay - g_data.f_lin_ay);
                g_data.f_lin_az += ACC_FILTER_ALPHA * (laz - g_data.f_lin_az);
            }
            g_data.qx = qx;
            g_data.qy = qy;
            g_data.qz = qz;
            g_data.qw = qw;
            g_data.roll  = wrap_pi(roll - g_roll0);
            g_data.pitch = pitch - g_pitch0;
            g_data.yaw   = wrap_pi(yaw - g_yaw0);
            g_data.fusion_ok = converged;
            taskEXIT_CRITICAL(&g_data_lock);
        }

        /* 10 lecturas fallidas seguidas (100 ms): la orientacion esta
         * congelada, se marca como no valida y el carro se detiene */
        if (fail_streak >= 10)
        {
            taskENTER_CRITICAL(&g_data_lock);
            g_data.fusion_ok = false;
            taskEXIT_CRITICAL(&g_data_lock);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

/* ============================================================
 * micro-ROS: entidades
 * ============================================================ */
static rcl_allocator_t   s_allocator;
static rcl_init_options_t s_init_options;
static rclc_support_t    s_support;
static bool              s_support_ok = false;
static rcl_node_t        s_node;
static rcl_timer_t       s_timer;
static rclc_executor_t   s_executor;

static rcl_publisher_t imu_raw_pub, imu_filtered_pub, rpy_pub;
static sensor_msgs__msg__Imu imu_raw_msg, imu_filtered_msg;
static geometry_msgs__msg__Vector3 rpy_msg;

static rcl_subscription_t zero_sub;
static std_msgs__msg__Empty zero_msg;
static rcl_subscription_t arm_sub;
static std_msgs__msg__Bool arm_msg;
static rcl_publisher_t armed_pub;
static std_msgs__msg__Bool armed_msg;

static char s_frame_id[] = "glove_imu";

static rcl_publisher_t *const s_pubs[] = { &imu_raw_pub, &imu_filtered_pub, &rpy_pub, &armed_pub };
#define NUM_PUBS (sizeof(s_pubs) / sizeof(s_pubs[0]))

static void zero_callback(const void *msgin)
{
    (void)msgin;
    g_zero_req = true;
}

static void set_armed(bool on, const char *who);

static void arm_callback(const void *msgin)
{
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    set_armed(m->data, "PC");
}

static void stamp_header(std_msgs__msg__Header *h, int64_t ns)
{
    h->stamp.sec = (int32_t)(ns / 1000000000LL);
    h->stamp.nanosec = (uint32_t)(ns % 1000000000LL);
    h->frame_id.data = s_frame_id;
    h->frame_id.size = strlen(s_frame_id);
    h->frame_id.capacity = sizeof(s_frame_id);
}

static void publish_timer_callback(rcl_timer_t *timer, int64_t last_call_time, uintptr_t user_data)
{
    (void)last_call_time;
    (void)user_data;
    if (timer == NULL) return;

    glove_data_t d;
    taskENTER_CRITICAL(&g_data_lock);
    d = g_data;
    taskEXIT_CRITICAL(&g_data_lock);

    /* Hora del agente si se sincronizo; si no, tiempo desde el arranque */
    int64_t ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos()
                                               : esp_timer_get_time() * 1000LL;

    stamp_header(&imu_raw_msg.header, ns);
    imu_raw_msg.orientation_covariance[0] = -1.0;   /* sin orientacion en raw */
    imu_raw_msg.linear_acceleration.x = d.raw_ax;
    imu_raw_msg.linear_acceleration.y = d.raw_ay;
    imu_raw_msg.linear_acceleration.z = d.raw_az;
    imu_raw_msg.angular_velocity.x = d.raw_gx;
    imu_raw_msg.angular_velocity.y = d.raw_gy;
    imu_raw_msg.angular_velocity.z = d.raw_gz;
    RCSOFTCHECK(rcl_publish(&imu_raw_pub, &imu_raw_msg, NULL));

    stamp_header(&imu_filtered_msg.header, ns);
    imu_filtered_msg.orientation.x = d.qx;
    imu_filtered_msg.orientation.y = d.qy;
    imu_filtered_msg.orientation.z = d.qz;
    imu_filtered_msg.orientation.w = d.qw;
    imu_filtered_msg.linear_acceleration.x = d.f_lin_ax;   /* sin gravedad */
    imu_filtered_msg.linear_acceleration.y = d.f_lin_ay;
    imu_filtered_msg.linear_acceleration.z = d.f_lin_az;
    imu_filtered_msg.angular_velocity.x = d.f_gx;
    imu_filtered_msg.angular_velocity.y = d.f_gy;
    imu_filtered_msg.angular_velocity.z = d.f_gz;
    RCSOFTCHECK(rcl_publish(&imu_filtered_pub, &imu_filtered_msg, NULL));

    rpy_msg.x = d.roll;
    rpy_msg.y = d.pitch;
    rpy_msg.z = d.yaw;
    RCSOFTCHECK(rcl_publish(&rpy_pub, &rpy_msg, NULL));

    static int armed_div = 0;
    if (++armed_div >= 5)   /* 10 Hz */
    {
        armed_div = 0;
        armed_msg.data = g_armed;
        RCSOFTCHECK(rcl_publish(&armed_pub, &armed_msg, NULL));
    }
}

#define ENTITY_CREATION_TIMEOUT_MS  3000

static bool create_entities(void)
{
    for (size_t i = 0; i < NUM_PUBS; i++) *s_pubs[i] = rcl_get_zero_initialized_publisher();
    zero_sub = rcl_get_zero_initialized_subscription();
    arm_sub = rcl_get_zero_initialized_subscription();
    s_node = rcl_get_zero_initialized_node();
    s_timer = rcl_get_zero_initialized_timer();
    s_executor = rclc_executor_get_zero_initialized_executor();
    s_support_ok = false;

    RCRETCHECK(rclc_support_init_with_options(&s_support, 0, NULL, &s_init_options, &s_allocator));
    s_support_ok = true;

    (void)rmw_uros_set_context_entity_creation_session_timeout(
        rcl_context_get_rmw_context(&s_support.context), ENTITY_CREATION_TIMEOUT_MS);

    RCRETCHECK(rclc_node_init_default(&s_node, "glove_esp32_node", "", &s_support));

    /* Best effort: se publica muchas veces por segundo, perder uno no
     * importa y no bloquea esperando acuses (ver main.c del carro). */
    RCRETCHECK(rclc_publisher_init_best_effort(&imu_raw_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "glove/imu/raw"));
    RCRETCHECK(rclc_publisher_init_best_effort(&imu_filtered_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "glove/imu/filtered"));
    RCRETCHECK(rclc_publisher_init_best_effort(&rpy_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "glove/rpy"));
    RCRETCHECK(rclc_publisher_init_best_effort(&armed_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "glove/armed"));

    RCRETCHECK(rclc_subscription_init_default(&zero_sub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "glove/zero"));
    RCRETCHECK(rclc_subscription_init_default(&arm_sub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "glove/arm"));

    RCRETCHECK(rclc_timer_init_default2(&s_timer, &s_support, RCL_MS_TO_NS(1000 / GLOVE_PUBLISH_HZ),
                                        publish_timer_callback, true));

    RCRETCHECK(rclc_executor_init(&s_executor, &s_support.context, 3, &s_allocator));
    RCRETCHECK(rclc_executor_add_subscription(&s_executor, &zero_sub, &zero_msg,
                                              &zero_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&s_executor, &arm_sub, &arm_msg,
                                              &arm_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_timer(&s_executor, &s_timer));

    /* Reloj comun con el PC para los header.stamp */
    if (rmw_uros_sync_session(1000) != RMW_RET_OK)
    {
        ESP_LOGW(TAG, "No se pudo sincronizar el reloj con el agente; stamps relativos al arranque");
    }

    return true;
}

static void destroy_entities(void)
{
    if (s_support_ok)
    {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&s_support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }

    for (size_t i = 0; i < NUM_PUBS; i++) RCSOFTCHECK(rcl_publisher_fini(s_pubs[i], &s_node));
    RCSOFTCHECK(rcl_subscription_fini(&zero_sub, &s_node));
    RCSOFTCHECK(rcl_subscription_fini(&arm_sub, &s_node));
    RCSOFTCHECK(rcl_timer_fini(&s_timer));
    RCSOFTCHECK(rclc_executor_fini(&s_executor));
    RCSOFTCHECK(rcl_node_fini(&s_node));
    if (s_support_ok)
    {
        RCSOFTCHECK(rclc_support_fini(&s_support));
        s_support_ok = false;
    }
}

/* ============================================================
 * Mando directo por ESP-NOW
 * ============================================================ */
static void set_armed(bool on, const char *who)
{
    if (on && !g_data.fusion_ok)
    {
        ESP_LOGW(TAG, "No se puede armar: el MPU6050 aun no entrega orientacion (calibrando o sin lecturas)");
        return;
    }
    if (on != g_armed)
    {
        g_armed = on;
        ESP_LOGI(TAG, "Mando directo %s (%s)", on ? "ARMADO" : "desarmado", who);
    }
#if STATUS_LED_GPIO >= 0
    gpio_set_level(STATUS_LED_GPIO, on ? 1 : 0);
#endif
}

/* Envia la orientacion de la mano al carro a 50 Hz. */
static void espnow_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t seq = 0, fails = 0;
    while (1)
    {
        glove_data_t d;
        taskENTER_CRITICAL(&g_data_lock);
        d = g_data;
        taskEXIT_CRITICAL(&g_data_lock);

        glove_packet_t pkt = {0};
        pkt.seq = seq++;
        pkt.t_ms = (uint32_t)(esp_timer_get_time() / 1000);
        pkt.flags = (g_armed ? GLOVE_FLAG_ARMED : 0) | (d.fusion_ok ? GLOVE_FLAG_FUSION_OK : 0);
        pkt.roll = d.roll;
        pkt.pitch = d.pitch;
        pkt.yaw = d.yaw;
        pkt.gx = d.f_gx;
        pkt.gy = d.f_gy;
        pkt.gz = d.f_gz;
        pkt.ax = d.f_lin_ax;
        pkt.ay = d.f_lin_ay;
        pkt.az = d.f_lin_az;
        if (espnow_link_send(&pkt) != ESP_OK && (++fails % 250) == 1)
        {
            ESP_LOGW(TAG, "ESP-NOW: fallo al enviar (%lu en total)", (unsigned long)fails);
        }
#if GLOVE_TELEMETRY
        /* t_ms,seq,armed,fusion,roll,pitch,yaw (deg),gx,gy,gz (rad/s),
         * ax,ay,az (m/s^2),qnorm,i2c_fail,tx_fail,calib (3000 = giroscopio calibrado) */
        {
            const float r2d = 180.0f / (float)M_PI;
            printf("G,%lu,%lu,%d,%d,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%lu,%lu,%d%d%d%d\n",
                   (unsigned long)pkt.t_ms, (unsigned long)pkt.seq, g_armed ? 1 : 0, d.fusion_ok ? 1 : 0,
                   d.roll * r2d, d.pitch * r2d, d.yaw * r2d, d.f_gx, d.f_gy, d.f_gz,
                   d.f_lin_ax, d.f_lin_ay, d.f_lin_az, g_qnorm,
                   (unsigned long)g_i2c_fail, (unsigned long)fails,
                   g_bias_ok ? 3 : 0, 0, 0, 0);
        }
#endif
#if GLOVE_DEBUG_LOG
        if (seq % 50 == 0)   /* cada 1 s */
        {
            ESP_LOGI(TAG, "ESP-NOW -> carro: %s | roll %.1f pitch %.1f deg | canal %u",
                     g_armed ? "ARMADO" : "desarmado (pulsa BOOT)",
                     d.roll * 180.0f / (float)M_PI, d.pitch * 180.0f / (float)M_PI, espnow_link_channel());
        }
#endif
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ESPNOW_PERIOD_MS));
    }
}

/* Boton BOOT: corta = armar/desarmar, larga = postura neutra. */
static void button_task(void *arg)
{
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << ARM_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&in);
#if STATUS_LED_GPIO >= 0
    gpio_config_t out = { .pin_bit_mask = 1ULL << STATUS_LED_GPIO, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&out);
    gpio_set_level(STATUS_LED_GPIO, 0);
#endif

    int held_ms = 0;
    bool long_done = false;
    while (1)
    {
        bool pressed = gpio_get_level(ARM_BUTTON_GPIO) == 0;
        if (pressed)
        {
            held_ms += 20;
            if (!long_done && held_ms >= LONG_PRESS_MS)
            {
                long_done = true;
                g_zero_req = true;
                ESP_LOGI(TAG, "Boton: postura neutra");
#if STATUS_LED_GPIO >= 0
                for (int i = 0; i < 3; i++)
                {
                    gpio_set_level(STATUS_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(80));
                    gpio_set_level(STATUS_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(80));
                }
                gpio_set_level(STATUS_LED_GPIO, g_armed ? 1 : 0);
#endif
            }
        }
        else
        {
            if (held_ms >= 40 && !long_done)   /* pulsacion corta (con antirrebote) */
            {
                set_armed(!g_armed, "boton");
            }
            held_ms = 0;
            long_done = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================================================
 * micro-ROS: maquina de estados de conexion (igual que el carro:
 * si se cae el agente o el WiFi, se destruye todo y se reconecta)
 * ============================================================ */
typedef enum
{
    AGENT_WAITING,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED,
} agent_state_t;

void micro_ros_task(void *arg)
{
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    esp_err_t ps_ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "No se pudo desactivar el ahorro de energia WiFi: %s", esp_err_to_name(ps_ret));
    }

    /* ESP-NOW necesita el WiFi ya iniciado; comparte radio y canal con
     * micro-ROS. Envia aunque el agente del PC no este disponible. */
    esp_err_t en = espnow_link_init(NULL);
    if (en == ESP_OK)
    {
        g_espnow_ok = true;
        xTaskCreate(espnow_tx_task, "espnow_tx", 3072, NULL, 5, NULL);
    }
    else
    {
        ESP_LOGE(TAG, "ESP-NOW no inicio: %s (el carro no recibira el mando directo)", esp_err_to_name(en));
    }

    s_allocator = rcl_get_default_allocator();
    s_init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&s_init_options, s_allocator) != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "rcl_init_options_init fallo");
        vTaskDelete(NULL);
    }

    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&s_init_options);
    if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                         CONFIG_MICRO_ROS_AGENT_PORT,
                                         rmw_options) != RMW_RET_OK)
    {
        ESP_LOGE(TAG, "Direccion del agente invalida");
        vTaskDelete(NULL);
    }

    agent_state_t state = AGENT_WAITING;
    TickType_t last_ping = 0;
    int wait_log = 0;
    int discover_tries = 0;
    int ping_failures = 0;

    while (1)
    {
        switch (state)
        {
        case AGENT_WAITING:
            if (rmw_uros_ping_agent_options(200, 1, rmw_options) == RMW_RET_OK)
            {
                state = AGENT_AVAILABLE;
                discover_tries = 0;
            }
            else
            {
                /* Sin respuesta en la IP configurada (~3 s): buscar el agente
                 * en la red. El router puede cambiarle la IP al PC (DHCP);
                 * el agente responde si se lanzo con --discovery 7400. */
                if (++discover_tries >= 6)
                {
                    discover_tries = 0;
                    if (rmw_uros_discover_agent(rmw_options) == RMW_RET_OK)
                    {
                        ESP_LOGI(TAG, "Agente micro-ROS encontrado en la red por descubrimiento");
                        break;
                    }
                }
                if (++wait_log >= 5)
                {
                    wait_log = 0;
                    ESP_LOGW(TAG, "Esperando al agente micro-ROS (IP configurada %s, buscando en la red)...",
                             CONFIG_MICRO_ROS_AGENT_IP);
                }
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            break;

        case AGENT_AVAILABLE:
            if (create_entities())
            {
                ESP_LOGI(TAG, "Agente conectado, entidades creadas");
                last_ping = xTaskGetTickCount();
                ping_failures = 0;
                state = AGENT_CONNECTED;
            }
            else
            {
                ESP_LOGW(TAG, "Fallo creando entidades, reintentando");
                destroy_entities();
                vTaskDelay(pdMS_TO_TICKS(500));
                state = AGENT_WAITING;
            }
            break;

        case AGENT_CONNECTED:
            if (xTaskGetTickCount() - last_ping > pdMS_TO_TICKS(1000))
            {
                last_ping = xTaskGetTickCount();
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
                {
                    if (++ping_failures >= 5)
                    {
                        ESP_LOGW(TAG, "Se perdio el agente micro-ROS");
                        state = AGENT_DISCONNECTED;
                        break;
                    }
                }
                else
                {
                    ping_failures = 0;
                }
            }
            RCSOFTCHECK(rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(5)));
            vTaskDelay(pdMS_TO_TICKS(5));
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            state = AGENT_WAITING;
            break;
        }
    }
}

/* ============================================================
 * app_main
 * ============================================================ */
void app_main(void)
{
    esp_err_t nvs_ret = nvs_flash_init();
    if ((nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES) || (nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    mpu_setup();

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2560, NULL, 3, NULL);
    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 4, NULL);
}
