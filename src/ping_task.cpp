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



/* UTILS */

static void write_u16_le(uint8_t *buf, uint16_t val)
{
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
}



/* PING SEND COMMANDS */
// 1006 set_ping_enable

// 1212 Distance
    


// these will parse the uart_transaction_t payload as their input buf and output to their corresponding struct
/* PARSING RECEIVED DATA*/

// 1212 distance response
static bool parse_distance(const uint8_t *p, size_t len, ping_distance_t *ping_distance_response)
{

    // notice little‑endian decoding
    uint32_t distance_mm        = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                                ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);

    uint16_t confidence         = (uint16_t)p[4] | ((uint16_t)p[5] << 8);

    uint16_t transmit_duration  = (uint16_t)p[6] | ((uint16_t)p[7] << 8);

    uint32_t ping_number        = (uint32_t)p[8] | ((uint32_t)p[9] << 8) |
                                ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);

    uint32_t scan_start_mm      = (uint32_t)p[12] | ((uint32_t)p[13] << 8) |
                                ((uint32_t)p[14] << 16) | ((uint32_t)p[15] << 24);

    uint32_t scan_length_mm     = (uint32_t)p[16] | ((uint32_t)p[17] << 8) |
                                ((uint32_t)p[18] << 16) | ((uint32_t)p[19] << 24);

    uint32_t gain_setting       = (uint32_t)p[20] | ((uint32_t)p[21] << 8) |
                                ((uint32_t)p[22] << 16) | ((uint32_t)p[23] << 24);

    ESP_LOGI(TAG, "Distance: %.3f m", distance_mm / 1000.0f);
    ESP_LOGI(TAG, "Confidence: %d %%", confidence);
    ESP_LOGI(TAG, "Transmit duration: %d us", transmit_duration);
    ESP_LOGI(TAG, "Ping number: %u", ping_number);
    ESP_LOGI(TAG, "Scan start: %u mm", scan_start_mm);
    ESP_LOGI(TAG, "Scan length: %u mm", scan_length_mm);
    ESP_LOGI(TAG, "Gain setting: %u", gain_setting);

    return 1;
}

// FREE RTOS TASK

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
    uart_transaction_t trans;
    ping_distance_t ping_distance_response;
    
    send_general_request(4, &trans);   // device_info

    if (trans.rx_len <= 0) {
        ESP_LOGE(TAG, "No response during init");
    } else {
        ESP_LOGI(TAG, "Ping1D responded (%d bytes)", trans.rx_len);
    }
    
    set_speed_of_sound(343.0f, &trans); // 343 m/s at ~20°C
    vTaskDelay(pdMS_TO_TICKS(50));
   

    while (1)
    {
        send_general_request(1212, &trans);   // distance

        if (trans.rx_len > 0)
        {
            parser.reset();

            for (int i = 0; i < trans.rx_len; i++)
            {
                auto state = parser.parseByte(trans.rx_buf[i]);

                if (state == PingParser::State::NEW_MESSAGE)
                {
                    uint16_t msg_id = parser.rxMessage.message_id();
                    uint8_t *p = parser.rxMessage.payload_data();
                
                    if (msg_id == 1212)
                    {
                        parse_distance(p, parser.rxMessage.payload_length(), &ping_distance_response);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "No response");
                    }
                }
                    
            }
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