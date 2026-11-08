#ifndef ESPNOW_LINK_H
#define ESPNOW_LINK_H

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OJO: este componente esta copiado en los dos proyectos
 *   ~/laboratorio_final_senales/components/EspNowLink  (carro)
 *   ~/laboratorio_final_guante/components/EspNowLink   (guante)
 * Si cambias el paquete, copialo igual en ambos y sube GLOVE_PKT_VERSION
 * (el carro descarta paquetes de otra version).
 *
 * Paquete guante -> carro por ESP-NOW (50 Hz, broadcast).
 * Angulos relativos a la postura neutra del operador (cero del guante).
 */
#define GLOVE_PKT_MAGIC        0x474C   /* "GL" */
#define GLOVE_PKT_VERSION      1

#define GLOVE_FLAG_ARMED       0x01     /* el operador habilito el mando directo */
#define GLOVE_FLAG_FUSION_OK   0x02     /* la fusion del BNO055 ya entrega orientacion */

typedef struct __attribute__((packed))
{
    uint16_t magic;
    uint8_t  version;
    uint8_t  group;
    uint8_t  flags;
    uint8_t  reserved;
    uint32_t seq;
    uint32_t t_ms;           /* reloj del guante */
    float roll, pitch, yaw;  /* rad, relativos a la postura neutra */
    float gx, gy, gz;        /* rad/s, giroscopio filtrado */
    float ax, ay, az;        /* m/s^2, aceleracion lineal filtrada */
} glove_packet_t;

/* Se llama desde la tarea de WiFi: debe ser corta (copiar y salir). */
typedef void (*espnow_link_rx_cb_t)(const glove_packet_t *pkt, int rssi);

/**
 * @brief Inicia ESP-NOW. Llamar despues de esp_wifi_start() (en estos
 *        proyectos: despues de uros_network_interface_initialize()).
 *        Si el WiFi no quedo conectado a un router, fija el canal
 *        CONFIG_ESPNOW_FALLBACK_CHANNEL.
 * @param rx_cb callback de recepcion (NULL en el guante, que solo envia).
 */
esp_err_t espnow_link_init(espnow_link_rx_cb_t rx_cb);

/** @brief Completa magic/version/grupo y envia el paquete por broadcast. */
esp_err_t espnow_link_send(glove_packet_t *pkt);

/** @brief Canal WiFi en uso (para el log y el tablero). */
uint8_t espnow_link_channel(void);

#ifdef __cplusplus
}
#endif

#endif
