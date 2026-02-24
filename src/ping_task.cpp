#include "ping_task.h"
#include "uart_manager.h"
#include "config.h"
#include "esp_timer.h"
#include <string.h>
#include "aggregator.h"
#include "esp_log.h"

// https://docs.bluerobotics.com/ping-protocol/
// https://docs.bluerobotics.com/ping-protocol/pingmessage-common/
// https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/

static QueueHandle_t ping_queue;

QueueHandle_t get_ping_queue()
{
    return ping_queue;
}



/* UTILS */

static void write_u16_le(uint8_t *buf, uint16_t val)
{
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
}

static uint16_t compute_checksum(const uint8_t *buf, size_t len)
{
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    return (uint16_t)sum;
}   

static size_t build_ping_command(uint8_t *buf, size_t bufsize, uint16_t msg_id, const uint8_t *payload, size_t payload_len)
{
     size_t total = 1+1+2+2+1+1+payload_len+2; // start1 + start2 + payload len + msg_id + src_id + dev_id + payload + checksum    
    if (bufsize < total) return 0;

    size_t i = 0;

    // write start bytes
    buf[i++] = 0x42;
    buf[i++] = 0x52;
    // write payload length
    write_u16_le(&buf[i], payload_len);
    i += 2;
    // write message ID
    write_u16_le(&buf[i], msg_id);
    i += 2;

    buf[i++] = 0x00; // src_id (host)
    buf[i++] = 0x00; // dev_id (ping)

    // write payload
    if (payload_len > 0)
    {
        memcpy(&buf[i], payload, payload_len);
        i += payload_len;
    }
    // write checksum
    uint16_t checksum = compute_checksum(buf, i);
    write_u16_le(&buf[i], checksum);
    i += 2;
    return i;
}

/* PING SEND COMMANDS */
// 1212 Distance
static size_t ping_distance(uint8_t *buf, size_t size)
{
    uint8_t payload[2];
    write_u16_le(payload, 1212); // distance request

    // 6 is the general request, payload is requesting a 1212 response
    return build_ping_command(buf, size, 6, payload, sizeof(payload));
}


// these will parse the uart_transaction_t payload as their input buf and output to their corresponding struct
/* PARSING RECEIVED DATA*/

// 1212 distance response
static bool parse_distance(const uint8_t *buf, size_t len, ping_distance_t *distance_response)
{
    if (len < 24) return false;
    if (buf[0] != 0x42 || buf[1] != 0x52) return false; // check start bytes

    uint16_t payload_len = buf[2] | (buf[3] << 8);
    uint16_t msg_id = buf[4] | (buf[5] << 8);

    if (msg_id != 1212) return false; // check message ID

    size_t expected_len = 1+1+2+2+1+1+payload_len+2;
    if (len < expected_len) return false; // check total length

    // check checksum
    uint16_t recv_checksum = buf[expected_len - 2] | (buf[expected_len - 1] << 8);
    uint16_t calc_checksum = compute_checksum(buf, expected_len - 2);
    if (recv_checksum != calc_checksum) return false;
 
    // Ping distance format:
    // u32 distance_mm
    // u16 confidence
    // u16 transmit_duration
    // u32 ping_number
    // u32 scan_start
    // u32 scan_length
    // u32 gain_setting
    uint32_t distance_mm = buf[8] | (buf[9] << 8) | (buf[10] << 16) | (buf[11] << 24);
    uint16_t confidence = buf[12] | (buf[13] << 8   );
    uint16_t transmit_duration = buf[14] | (buf[15] << 8);
    uint32_t ping_number = buf[16] | (buf[17] << 8) | (buf[18] << 16) | (buf[19] << 24);
    uint32_t scan_start = buf[20] | (buf[21] << 8) | (buf[22] << 16) | (buf[23] << 24);
    uint32_t scan_length = buf[24] | (buf[25] << 8) | (buf[26] << 16) | (buf[27] << 24);
    uint32_t gain_setting = buf[28] | (buf[29] << 8) | (buf[30] << 16) | (buf[31] << 24);

    distance_response->distance_mm = distance_mm;
    distance_response->confidence = confidence;
    distance_response->transmit_duration_us = transmit_duration;
    distance_response->ping_number = ping_number;
    distance_response->scan_start_mm = scan_start;
    distance_response->scan_length_mm = scan_length;
    distance_response->gain_setting = gain_setting;
    distance_response->timestamp = xTaskGetTickCount();

    return true;
}


// FREE RTOS TASK

static void ping_task(void *arg)
{
    uart_transaction_t trans;
    ping_distance_t distance_response;

    while (1)
    {
        // Build Ping command

        // cmd will be our full ping protocol message

        memset(&trans, 0, sizeof(trans));
        trans.device = PING;

        trans.tx_len = ping_distance(trans.tx_buf, sizeof(trans.tx_buf));

        trans.timeout_ms = 200;
        trans.caller = xTaskGetCurrentTaskHandle();
        ESP_LOGI("PING_TASK", "Requesting distance measurement...");
        xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // TODO: Parse Ping Response with funcs below
        parse_distance(trans.rx_buf, trans.rx_len, &distance_response);
        ESP_LOGI("PING_TASK", "Parsed Response!");
        // publish to ping queue?
        xQueueSend(ping_queue, &distance_response, 0);

        // send to SD task
        xQueueSend(get_record_queue(), &distance_response, 0);

        vTaskDelay(pdMS_TO_TICKS(g_sample_interval_ms));
    }
}

void init_ping_task()
{
    ping_queue = xQueueCreate(10, sizeof(ping_distance_t));
    xTaskCreatePinnedToCore(
        ping_task,
        "ping",
        4096,
        NULL,
        8,
        NULL,
        0
    );
}