#include "ping_task.h"
#include "uart_manager.h"
#include "config.h"
#include "esp_timer.h"
#include <string.h>
#include "aggregator.h"

// https://docs.bluerobotics.com/ping-protocol/
// https://docs.bluerobotics.com/ping-protocol/pingmessage-common/
// https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/

static void ping_task(void *arg)
{
    uart_transaction_t trans;
    ping_data_t ping;

    while (1)
    {
        // Build Ping command

        // cmd will be our full ping protocol message

        memset(&trans, 0, sizeof(trans));
        trans.device = PING;
        memcpy(trans.tx_buf, cmd, strlen(cmd));
        trans.tx_len = strlen(cmd);
        trans.timeout_ms = 200;
        trans.caller = xTaskGetCurrentTaskHandle();

        xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // TODO: Parse Ping Response with funcs below

        // publish to ping queue?

        // send to SD task
        xQueueSend(get_record_queue(), &ping, 0);

        vTaskDelay(pdMS_TO_TICKS(g_sample_interval_ms));
    }
}

void init_ping_task()
{
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

/* PING COMMANDS */
// 1212 Distance
static size_t distance(uint8_t *buf, size_t size)
{
    uint8_t payload[2];
    write_u16_le(payload, 1212); // distance request

    return build_ping_command(buf, size, 1212, payload, sizeof(payload));
}

// TODO: parse the full response and grab the buf, buf len



// these will parse the payload as the input buf
/* PARSING RECEIVED DATA*/

static bool parse_distance(const uint8_t *buf, size_t len, ping_data_t *ping)
{
    if (len < 24) return false;
    uf (buf[0])
}