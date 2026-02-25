#include "ping_task.h"
#include "uart_manager.h"
#include "config.h"
#include "esp_timer.h"
#include <string.h>
#include "aggregator.h"
#include "esp_log.h"
#include "ping-message.h"
#include "ping-parser.h"

// https://docs.bluerobotics.com/ping-protocol/
// https://docs.bluerobotics.com/ping-protocol/pingmessage-common/
// https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/

static QueueHandle_t ping_queue;
static const char *TAG = "PING_TASK";

QueueHandle_t get_ping_queue()
{
    return ping_queue;
}
static bool parse_distance(const uint8_t *buf, size_t len, void* out_struct);
static const ping_dispatch_entry_t ping_dispatch_table[] = {
    //{1007, parse_ack, sizeof(ping_enable_t)},
    {1212, parse_distance, sizeof(ping_distance_t)},

};

static const size_t ping_dispatch_table_size = sizeof(ping_dispatch_table)/sizeof(ping_dispatch_table[0]);

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
// 1006 set_ping_enable
static size_t ping_enable(uint8_t *buf, size_t size, uint8_t enable)
{
    uint8_t payload[1];
    payload[0] = enable;

    return build_ping_command(buf, size, 1006, payload, sizeof(payload));
}
// 1212 Distance
static size_t ping_distance(uint8_t *buf, size_t size)
{
    uint8_t payload[2];
    write_u16_le(payload, 1208); // request 1212 distance message
    return build_ping_command(buf, size, 6, payload, sizeof(payload));
}


static size_t ping_get_device_info(uint8_t *buf, size_t size)
{
    return build_ping_command(buf, size, 4, NULL, 0); // msg ID 4, no payload
}
    


// these will parse the uart_transaction_t payload as their input buf and output to their corresponding struct
/* PARSING RECEIVED DATA*/

// 1212 distance response
static bool parse_distance(const uint8_t *buf, size_t len, void* out_struct)
{
    ping_distance_t *distance_response = (ping_distance_t*)out_struct;
    
    for (size_t i = 0; i < len; i++)
    {
        ESP_LOGI("PING_PARSE", "RX: %02X", buf[i]);
    }
    

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

    ESP_LOGI("PING_PARSE", "Distance: %u mm, Confidence: %u, Duration: %u us, Ping #: %u",
             distance_response->distance_mm,
             distance_response->confidence,
             distance_response->transmit_duration_us,
             distance_response->ping_number);
    ESP_LOGI("PING_PARSE", "Scan start: %u mm, Scan length: %u mm, Gain: %u",
             distance_response->scan_start_mm,
             distance_response->scan_length_mm,
             distance_response->gain_setting);

    return true;
}


// FREE RTOS TASK
/*
static void ping_task(void *arg)
{
    uart_transaction_t trans;
    ping_distance_t distance_response;
    int enabled = 0;
    while (1)
    {
        // Build Ping command

        // cmd will be our full ping protocol message

        memset(&trans, 0, sizeof(trans));
        trans.device = PING;

        if (!enabled) {
            trans.tx_len = ping_enable(trans.tx_buf, sizeof(trans.tx_buf), true);
            enabled++;
        }
        else if (enabled == 1) {
            //trans.tx_len = ping_distance(trans.tx_buf, sizeof(trans.tx_buf));
            trans.tx_len = ping_get_device_info(trans.tx_buf, sizeof(trans.tx_buf));
            enabled++;
        }
        else {
            trans.tx_len = ping_distance(trans.tx_buf, sizeof(trans.tx_buf));
        }

        trans.timeout_ms = 500;
        trans.caller = xTaskGetCurrentTaskHandle();
        ESP_LOGI("PING_TASK", "Requesting distance measurement...");
        xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // TODO: Parse Ping Response with funcs below
        bool parse_ok = parse_distance(trans.rx_buf, trans.rx_len, &distance_response);
        ESP_LOGI("PING_TASK", "Received response, length %d, parse_ok %d", trans.rx_len, parse_ok);
        if (parse_ok) {
            ESP_LOGI("PING_TASK", "Parsed Response!");
            // publish to ping queue?
            xQueueSend(ping_queue, &distance_response, 0);

            // send to SD task
            xQueueSend(get_record_queue(), &distance_response, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(g_sample_interval_ms));
    }
}
    */
static void send_general_request(uint16_t requested_id, uart_transaction_t *trans)
{
    ping_message msg(64);   // safe buffer size

    msg.set_message_id(6);      // general_request
    msg.set_source_device_id(0);
    msg.set_destination_device_id(0);
    msg.set_payload_length(2);

    // little endian
    msg.msgData[ping_message::headerLength + 0] = requested_id & 0xFF;
    msg.msgData[ping_message::headerLength + 1] = (requested_id >> 8) & 0xFF;

    msg.updateChecksum();
    ESP_LOGI(TAG, "MSG BUILT");

    memset(trans, 0, sizeof(*trans));
    trans->device = PING;
    memcpy(trans->tx_buf, msg.msgData, msg.msgDataLength());
    trans->tx_len = msg.msgDataLength();
    trans->timeout_ms = 500;
    trans->caller = xTaskGetCurrentTaskHandle();

    ESP_LOGI("PING_TASK", "Requesting distance measurement...");
    xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /*
    uart_write_bytes(UART_PORT,
                     (const char*)msg.msgData,
                     msg.msgDataLength());
                     */
}
static void set_speed_of_sound(float sos_m_s, uart_transaction_t *trans)
{
    // Ping1D expects speed of sound in hundredths of m/s
    // e.g., 343.0 m/s -> 34300
    uint16_t sos_val = (uint16_t)(sos_m_s * 100);

    ping_message msg(64);
    msg.set_message_id(6);       // general request
    msg.set_source_device_id(0);
    msg.set_destination_device_id(0);
    msg.set_payload_length(3);   // 2 bytes for sos + 1 byte setting ID

    // first byte: setting type (2 for speed of sound)
    msg.msgData[ping_message::headerLength + 0] = 2;  

    // next 2 bytes: speed of sound (little endian)
    msg.msgData[ping_message::headerLength + 1] = sos_val & 0xFF;
    msg.msgData[ping_message::headerLength + 2] = (sos_val >> 8) & 0xFF;

    msg.updateChecksum();

    memset(trans, 0, sizeof(*trans));
    trans->device = PING;
    memcpy(trans->tx_buf, msg.msgData, msg.msgDataLength());
    trans->tx_len = msg.msgDataLength();
    trans->timeout_ms = 500;
    trans->caller = xTaskGetCurrentTaskHandle();
    ESP_LOGI("PING_TASK", "Set speed of sound to %.2f m/s", sos_m_s);
    xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

}

void ping_task(void *arg)
{
    PingParser parser;

    ESP_LOGI(TAG, "Syncing with Ping1D...");

    /* ---- Step 1: Request device info to confirm comms ---- */
    uart_transaction_t trans;
    send_general_request(1, &trans);   // device_information
    ESP_LOGI(TAG, "passed req");
    
    if (trans.rx_len <= 0) {
        ESP_LOGE(TAG, "No response during init");
    } else {
        ESP_LOGI(TAG, "Ping1D responded (%d bytes)", trans.rx_len);
    }
    
    
    ESP_LOGI(TAG, "Setting speed of sound...");
    set_speed_of_sound(343.0f, &trans); // 343 m/s at ~20°C
    vTaskDelay(pdMS_TO_TICKS(50));
   
    /* ---- Main Loop ---- */
    while (1)
    {
        send_general_request(1212, &trans);   // distance
        ESP_LOGI(TAG, "BACK FROM REQUEST");
        //for (int i = 0; i < trans.rx_len; i++)
        //{
         //   ESP_LOGI("PING_TASK", "RX: %02X", trans.rx_buf[i]);
        //}
        if (trans.rx_len > 0)
        {
            parser.reset();

            for (int i = 0; i < trans.rx_len; i++)
            {
                auto state = parser.parseByte(trans.rx_buf[i]);

                if (state == PingParser::State::NEW_MESSAGE)
                {
                    uint16_t msg_id = parser.rxMessage.message_id();

                    if (msg_id == 1212)
                    {
                        uint8_t *p = parser.rxMessage.payload_data();
/*
                        uint32_t distance_mm =
                            p[0] |
                            (p[1] << 8) |
                            (p[2] << 16) |
                            (p[3] << 24);
*/
                        uint16_t distance_mm = p[0] | (p[1] << 8);
                        ESP_LOGI(TAG,
                                 "Distance: %.3f m",
                                 distance_mm / 1000.0f);
                        uint8_t confidence = p[2]; // or p[2] | (p[3] << 8) if 2 bytes
ESP_LOGI(TAG, "Confidence: %d", confidence);
                    }
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "No response");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void init_ping_task()
{
    ping_queue = xQueueCreate(10, sizeof(ping_distance_t));

    xTaskCreatePinnedToCore(
        ping_task,
        "ping",
        8192,
        NULL,
        8,
        NULL,
        0
    );
}