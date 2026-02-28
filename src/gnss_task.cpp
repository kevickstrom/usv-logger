#include "gnss_task.h"
#include "uart_manager.h"
#include "config.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <fcntl.h>
#include <nmea.h>
#include <gpgll.h>
#include <gpgga.h>
#include <gprmc.h>
#include <gpgsa.h>
#include <gpvtg.h>
#include <gptxt.h>
#include <gpgsv.h>

#include <string.h>

static QueueHandle_t gps_queue;

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

static void ubx_checksum(const uint8_t *buf, size_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < len; i++)
    {
        *ck_a += buf[i];
        *ck_b += *ck_a;
    }
}

static size_t build_ubx_frame(uint8_t *buf, size_t bufsize, uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, size_t payload_len)
{
    buf[0] = UBX_SYNC1;
    buf[1] = UBX_SYNC2;
    buf[2] = msg_class;
    buf[3] = msg_id;
    buf[4] = payload_len & 0xFF;
    buf[5] = (payload_len >> 8) & 0xFF;

    if (payload_len > 0)
    {
        memcpy(&buf[6], payload, payload_len);
    }

    uint8_t ck_a, ck_b;
    ubx_checksum(buf, 6 + payload_len, &ck_a, &ck_b);
    buf[6 + payload_len] = ck_a;
    buf[7 + payload_len] = ck_b;

    return 8 + payload_len;
}

static bool parse_nav_pvt(const uint8_t *buf, size_t len, gps_data_t *out_data)
{
    if (len < 92) return false;
    //if (buf[0] != UBX_SYNC1 || buf[1] != UBX_SYNC2) return false;
    //if (buf[2] != 0x01 || buf[3] != 0x07) return false; // NAV-PVT

    out_data->lat = *(int32_t*)(buf + 30) / 1e7;
    out_data->lon = *(int32_t*)(buf + 34) / 1e7;
    out_data->alt_m = *(int32_t*)(buf + 38) / 1000.0f;
    out_data->fix_type = buf[20];
    out_data->num_sats = buf[23];
    //out_data->timestamp = esp_get_time() / 1000;

    return true;
}

static void gnss_task(void *arg)
{
    uart_transaction_t trans;

    while (1)
    {
        memset(&trans, 0, sizeof(trans));
        trans.device = GPS;
        trans.timeout_ms = 100;
        trans.caller = xTaskGetCurrentTaskHandle();

        xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // TODO: Parse GNSS data

        vTaskDelay(pdMS_TO_TICKS(g_sample_interval_ms));
    }
}

QueueHandle_t get_gps_queue()
{
    return gps_queue;
}

void init_gnss_task()
{
    gps_queue = xQueueCreate(5, sizeof(gps_data_t));

    xTaskCreatePinnedToCore(
        gnss_task,
        "gnss",
        4096,
        NULL,
        9,
        NULL,
        0
    );
}