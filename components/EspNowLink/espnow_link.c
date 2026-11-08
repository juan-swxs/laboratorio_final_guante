#include "espnow_link.h"

#include <string.h>

#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

static const char *TAG = "ESPNOW";
static const uint8_t BROADCAST[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static espnow_link_rx_cb_t s_rx_cb = NULL;

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (s_rx_cb == NULL || len != (int)sizeof(glove_packet_t))
    {
        return;
    }
    glove_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic != GLOVE_PKT_MAGIC || pkt.version != GLOVE_PKT_VERSION ||
        pkt.group != (uint8_t)CONFIG_ESPNOW_GROUP_ID)
    {
        return;   /* otro grupo, otra version u otro tipo de paquete */
    }
    int rssi = (info && info->rx_ctrl) ? info->rx_ctrl->rssi : 0;
    s_rx_cb(&pkt, rssi);
}

uint8_t espnow_link_channel(void)
{
    uint8_t ch = 0;
    wifi_second_chan_t second;
    if (esp_wifi_get_channel(&ch, &second) != ESP_OK)
    {
        return 0;
    }
    return ch;
}

esp_err_t espnow_link_init(espnow_link_rx_cb_t rx_cb)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK)
    {
        /* Sin router: los dos ESP32 deben quedar en el mismo canal fijo */
        esp_err_t r = esp_wifi_set_channel(CONFIG_ESPNOW_FALLBACK_CHANNEL, WIFI_SECOND_CHAN_NONE);
        ESP_LOGW(TAG, "Sin router: ESP-NOW en el canal fijo %d (%s)",
                 CONFIG_ESPNOW_FALLBACK_CHANNEL, esp_err_to_name(r));
    }

    /* El ahorro de energia hace perder paquetes de ESP-NOW */
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BROADCAST, ESP_NOW_ETH_ALEN);
    peer.channel = 0;            /* canal actual */
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST)
    {
        return ret;
    }

    s_rx_cb = rx_cb;
    if (rx_cb != NULL)
    {
        ret = esp_now_register_recv_cb(on_recv);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    ESP_LOGI(TAG, "ESP-NOW listo: canal %u, grupo %d", espnow_link_channel(), CONFIG_ESPNOW_GROUP_ID);
    return ESP_OK;
}

esp_err_t espnow_link_send(glove_packet_t *pkt)
{
    pkt->magic = GLOVE_PKT_MAGIC;
    pkt->version = GLOVE_PKT_VERSION;
    pkt->group = (uint8_t)CONFIG_ESPNOW_GROUP_ID;
    return esp_now_send(BROADCAST, (const uint8_t *)pkt, sizeof(*pkt));
}
