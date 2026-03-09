#include "ping_task.h"
#include "uart_manager.h"
#include "config.h"
#include "esp_timer.h"
#include <string.h>
#include "aggregator.h"
#include "esp_log.h"
#include "ping-message.h"
#include "ping-parser.h"
#include "sd_task.h"
#include "lora_task.h"

// https://docs.bluerobotics.com/ping-protocol/
// https://docs.bluerobotics.com/ping-protocol/pingmessage-common/
// https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/

#define DEFAULT_BAUD 115200
static uint32_t default_timeout_ms = 200;
static QueueHandle_t ping_queue;
static const char *TAG = "PING_TASK";
static save_req_t save_req;
static lora_request_t lora_req;
static char lora_tx_static_buf[256];

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

    memset(trans, 0, sizeof(*trans));
    trans->device = PING;
    trans->baud = DEFAULT_BAUD;
    memcpy(trans->tx_buf, msg.msgData, msg.msgDataLength());
    trans->tx_len = msg.msgDataLength();
    trans->timeout_ms = default_timeout_ms;
    trans->caller = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG, "Requesting %d", requested_id);
    xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

}


/* PING SEND COMMANDS */

// 4 device_info
static void get_device_info(uart_transaction_t *trans)
{
    send_general_request(4, trans);
}

// 1001 set range
static void set_range(uint32_t scan_start_mm, uint32_t scan_length_mm, uart_transaction_t *trans)
{
    // Build Ping message
    ping_message msg(32); // enough buffer for header + 8 bytes payload + crc

    // This is a *set* message: ID = 1001
    msg.set_message_id(1001);
    msg.set_source_device_id(0);
    msg.set_destination_device_id(0);

    msg.set_payload_length(8); // two u32 fields

    // Write little-endian scan_start
    msg.msgData[ping_message::headerLength + 0] = (uint8_t)(scan_start_mm & 0xFF);
    msg.msgData[ping_message::headerLength + 1] = (uint8_t)((scan_start_mm >> 8) & 0xFF);
    msg.msgData[ping_message::headerLength + 2] = (uint8_t)((scan_start_mm >> 16) & 0xFF);
    msg.msgData[ping_message::headerLength + 3] = (uint8_t)((scan_start_mm >> 24) & 0xFF);

    // Write little-endian scan_length
    msg.msgData[ping_message::headerLength + 4] = (uint8_t)(scan_length_mm & 0xFF);
    msg.msgData[ping_message::headerLength + 5] = (uint8_t)((scan_length_mm >> 8) & 0xFF);
    msg.msgData[ping_message::headerLength + 6] = (uint8_t)((scan_length_mm >> 16) & 0xFF);
    msg.msgData[ping_message::headerLength + 7] = (uint8_t)((scan_length_mm >> 24) & 0xFF);

    // Fix checksum after setting payload
    msg.updateChecksum();

    memset(trans, 0, sizeof(*trans));
    trans->device = PING;
    trans->baud = DEFAULT_BAUD;
    memcpy(trans->tx_buf, msg.msgData, msg.msgDataLength());
    trans->tx_len = msg.msgDataLength();
    trans->timeout_ms = default_timeout_ms;
    trans->caller = xTaskGetCurrentTaskHandle();

    xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Sent set_range: start=%u mm, length=%u mm",
             scan_start_mm, scan_length_mm);
}

// 1002 set_speed_of_sound
static void set_speed_of_sound(float sos_val, uart_transaction_t *trans)
{
    // Ping1D expects speed of sound in mm/s

    ping_message msg(64);
    msg.set_message_id(1002);       // general request
    msg.set_source_device_id(0);
    msg.set_destination_device_id(0);
    msg.set_payload_length(4);   // 4 bytes - u32

    // first byte: setting type (2 for speed of sound)
    msg.msgData[ping_message::headerLength + 0] = 2;  

    // next 4 bytes: speed of sound (little endian)
    uint8_t *p = (uint8_t *)&sos_val;

    msg.msgData[ping_message::headerLength + 1] = p[0];
    msg.msgData[ping_message::headerLength + 2] = p[1];
    msg.msgData[ping_message::headerLength + 3] = p[2];
    msg.msgData[ping_message::headerLength + 4] = p[3];

    msg.updateChecksum();

    memset(trans, 0, sizeof(*trans));
    trans->device = PING;
    trans->baud = DEFAULT_BAUD;
    memcpy(trans->tx_buf, msg.msgData, msg.msgDataLength());
    trans->tx_len = msg.msgDataLength();
    trans->timeout_ms = default_timeout_ms;
    trans->caller = xTaskGetCurrentTaskHandle();

    xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "Set speed of sound to %.2f mm/s", sos_val);

}

// 1003 set_mode_auto
static void send_set_mode_auto(uint8_t mode_auto, uart_transaction_t *trans)
{
    // payload = 1 byte
    ping_message msg(16);

    msg.set_message_id(1003);
    msg.set_source_device_id(0);
    msg.set_destination_device_id(0);
    msg.set_payload_length(1);

    msg.msgData[ping_message::headerLength + 0] = mode_auto;

    msg.updateChecksum();

    memset(trans, 0, sizeof(*trans));
    trans->device = PING;
    trans->baud = DEFAULT_BAUD;
    memcpy(trans->tx_buf, msg.msgData, msg.msgDataLength());
    trans->tx_len = msg.msgDataLength();
    trans->timeout_ms = default_timeout_ms;
    trans->caller = xTaskGetCurrentTaskHandle();

    xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ESP_LOGI("PING_SEND", "Sent set_mode_auto: %u (%s)",
             mode_auto,
             mode_auto ? "AUTO" : "MANUAL");
}

// 1005 set_gain_setting
static void set_gain_setting(uint8_t gain_setting, uart_transaction_t *trans)
{
    // payload = 1 byte
    ping_message msg(16);

    msg.set_message_id(1005);
    msg.set_source_device_id(0);
    msg.set_destination_device_id(0);
    msg.set_payload_length(1);

    msg.msgData[ping_message::headerLength + 0] = gain_setting;

    msg.updateChecksum();

    memset(trans, 0, sizeof(*trans));
    trans->device = PING;
    trans->baud = DEFAULT_BAUD;
    memcpy(trans->tx_buf, msg.msgData, msg.msgDataLength());
    trans->tx_len = msg.msgDataLength();
    trans->timeout_ms = default_timeout_ms;
    trans->caller = xTaskGetCurrentTaskHandle();

    xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "set_gain_setting: %u", gain_setting);
}

// 1006 set_ping_enable
static void ping_enable(uint8_t enable, uart_transaction_t *trans)
{
    enable = (enable != 0);

    //send_general_request( ,trans);
}

// 1204 get range
static void get_range(uart_transaction_t *trans)
{
    send_general_request(1204, trans);
}

static void get_mode_auto(uart_transaction_t *trans)
{
    send_general_request(1205, trans);
}

// 1212 Distance
static void ping_distance(uart_transaction_t *trans)
{
    send_general_request(1212, trans);
}

// 1300 profile
static void send_profile_request( uart_transaction_t *trans,uint16_t profile_id = 0)
{
    ping_message msg(64);  // safe buffer size

    msg.set_message_id(1300);     // profile request
    msg.set_source_device_id(0);  // your host/device ID
    msg.set_destination_device_id(0);
    msg.set_payload_length(2);    // 2 bytes for profile ID if needed

    // little-endian profile ID
    msg.msgData[ping_message::headerLength + 0] = profile_id & 0xFF;
    msg.msgData[ping_message::headerLength + 1] = (profile_id >> 8) & 0xFF;

    msg.updateChecksum();

    memset(trans, 0, sizeof(*trans));
    trans->device = PING;
    trans->baud = DEFAULT_BAUD;
    memcpy(trans->tx_buf, msg.msgData, msg.msgDataLength());
    trans->tx_len = msg.msgDataLength();
    trans->timeout_ms = default_timeout_ms;
    trans->caller = xTaskGetCurrentTaskHandle();

    xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //xQueueSend(get_uart_queue(), &trans, pdMS_TO_TICKS(100));
    //ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));


    ESP_LOGI(TAG, "Requested Ping1D Profile", profile_id);

}



// these will parse the uart_transaction_t payload as their input buf and output to their corresponding struct
/* PARSING RECEIVED DATA*/

// Parse 1 ACK response
static bool parse_ack(const uint8_t *p, size_t len, ping_ack_t *ack)
{
    if (!p || !ack || len < 2) return false;

    // First 2 bytes = little-endian message ID that was acknowledged
    ack->acked_id = p[0] | ((uint16_t)p[1] << 8);

    ESP_LOGI(TAG, "Ping1D ACK for message ID: %u", ack->acked_id);

    return true;
}

// Parse 2 NACK response
static bool parse_nack(const uint8_t *p, size_t len, ping_nack_t *nack)
{
    if (!p || !nack || len < 2) return false;

    // First 2 bytes = little-endian message ID that caused the NACK
    nack->nacked_id = p[0] | ((uint16_t)p[1] << 8);

    // Copy the rest as ASCII text (truncate if longer than 128 bytes)
    size_t text_len = len - 2;
    if (text_len > sizeof(nack->nack_message) - 1) text_len = sizeof(nack->nack_message) - 1;

    memcpy(nack->nack_message, p + 2, text_len);
    nack->nack_message[text_len] = '\0';  // null-terminate

    ESP_LOGW(TAG, "Ping1D NACK for message ID: %u, message: %s", nack->nacked_id, nack->nack_message);

    return true;
}

// 4 Device Info
bool parse_device_info(const uint8_t *p, size_t len, ping_device_info_t *info)
{
    if (!p || !info || len < 6) return false; // adjust minimum length if necessary

    // Parse fields in little-endian
    info->device_id = p[0];
    info->device_type = p[1];
    info->firmware_version_major = p[2] | (p[3] << 8);
    info->firmware_version_minor = p[4] | (p[5] << 8);

    if (len >= 8) {
        info->voltage_5 = p[6] | (p[7] << 8);
    } else {
        info->voltage_5 = 0;
    }

    ESP_LOGI(TAG, "Device Info - ID: %u, Type: %u, FW: %u.%u, 5V: %u mV",
             info->device_id,
             info->device_type,
             info->firmware_version_major,
             info->firmware_version_minor,
             info->voltage_5);

    return true;
}

// Parse 1002 response
static bool parse_speed_of_sound(const uint8_t *p, size_t len, ping_speed_of_sound_t *sos_response)
{
    if (!p || !sos_response || len < 2) {
        return false; // invalid input
    }

    // Read 16-bit little-endian value
    uint16_t sos_hundredths = (uint16_t)p[0] | ((uint16_t)p[1] << 8);

    // Convert hundredths of m/s to m/s float
    sos_response->speed_of_sound = sos_hundredths / 100.0f;

    ESP_LOGI(TAG, "Speed of Sound: %.2f m/s", sos_response->speed_of_sound);

    return true;
}

// 1204 range
bool parse_range(const uint8_t *p, size_t len, ping_range_t *range)
{
    if (!p || !range || len < 8) return false;

    // Little-endian parse
    range->scan_start_mm  = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                            ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);

    range->scan_length_mm = (uint32_t)p[4] | ((uint32_t)p[5] << 8) |
                            ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);

    ESP_LOGI(TAG, "Range: scan_start: %u mm, scan_length: %u mm",
             range->scan_start_mm, range->scan_length_mm);

    return true;
}

// 1205 mode_auto
static bool parse_mode_auto(const uint8_t *p,size_t len, ping_mode_auto_t *mode)
{
    if (!p || !mode || len < 1)
        return false;

    mode->mode_auto = p[0];

    ESP_LOGI(TAG, "Mode Auto: %u (%s)",
             mode->mode_auto,
             mode->mode_auto ? "AUTO" : "MANUAL");

    return true;
}

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
 // 1300 pofile
 bool parse_profile(const uint8_t *p, size_t len, ping_profile_t *profile)
{
    // Need at least 28 bytes for the fixed fields
    if (!p || !profile || len < 28) {
        ESP_LOGW(TAG, "parse_profile: too short (%d bytes)", len);
        return false;
    }

    // Read fixed header fields (little endian)
    profile->distance_mm        = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    profile->confidence         = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
    profile->transmit_duration_us = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
    profile->ping_number        = (uint32_t)p[8] | ((uint32_t)p[9] << 8) | ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);
    profile->scan_start_mm      = (uint32_t)p[12] | ((uint32_t)p[13] << 8) | ((uint32_t)p[14] << 16) | ((uint32_t)p[15] << 24);
    profile->scan_length_mm     = (uint32_t)p[16] | ((uint32_t)p[17] << 8) | ((uint32_t)p[18] << 16) | ((uint32_t)p[19] << 24);
    profile->gain_setting       = (uint32_t)p[20] | ((uint32_t)p[21] << 8) | ((uint32_t)p[22] << 16) | ((uint32_t)p[23] << 24);

    // Next is profile_data_length (u16)
    uint16_t profile_data_len = (uint16_t)p[24] | ((uint16_t)p[25] << 8);

    // Check that len includes data + header
    size_t expected_min = 26 + profile_data_len;
    if (len < expected_min) {
        ESP_LOGW(TAG, "parse_profile: incomplete profile data (%d < %d)", len, expected_min);
        return false;
    }

    profile->profile_data_length = profile_data_len;
    profile->timestamp = esp_timer_get_time() / 1000; // optional timestamp

    // Copy the profile data
    if (profile_data_len > sizeof(profile->profile_data)) {
        // clamp if buffer is smaller
        profile_data_len = sizeof(profile->profile_data);
    }
    memcpy(profile->profile_data, &p[26], profile_data_len);

    ESP_LOGI(TAG, "Profile parsed: dist=%u mm conf=%u len=%u gain=%u", 
             profile->distance_mm, profile->confidence, profile->profile_data_length, profile->gain_setting);

    // for (size_t i = 0; i < profile_data_len; i++) {
    //     ESP_LOGI(TAG,"bin %zu: %u\n", i, profile->profile_data[i]);
    // }

    return true;
}

void make_csv(const ping_profile_t *p, char **csvln, size_t *csvlen)
{
    static char buffer[128];  // static storage, no malloc

    int len = snprintf(
    buffer,
    sizeof(buffer),
    "%lld,%lu,%lu,%u,%u,%lu,%lu,%lu",
    p->timestamp,
    p->ping_number,
    p->distance_mm,
    p->confidence,
    p->transmit_duration_us,
    p->scan_start_mm,
    p->scan_length_mm,
    p->gain_setting
);

    if (len < 0) {
        *csvln = NULL;
        *csvlen = 0;
        return;
    }

    if ((size_t)len >= sizeof(buffer)) {
        len = sizeof(buffer) - 1;  // truncated
    }

    *csvln = buffer;
    *csvlen = (size_t)len;
}

static void record_data(char *tosave)
{
    // file saving queue
    int len = snprintf(save_req.data, sizeof(save_req.data)+1,"%s\n",tosave);
    snprintf(save_req.fname, sizeof(save_req.fname), "%s", "ping_log.csv");
    save_req.device = PING;
    save_req.len = (len < sizeof(save_req.data)) ? len : sizeof(save_req.data);
    ESP_LOGI(TAG, "queued %lu bytes for file: %s", save_req.len, save_req.fname);

    xQueueSend(get_save_queue(), &save_req, portMAX_DELAY);
    


    // lora transmit queue
    lora_req.device = PING;
    lora_req.id = PING;
    lora_req.lora_tx_buf = lora_tx_static_buf;
    size_t tx_len = (save_req.len < sizeof(lora_tx_static_buf) - 1) ? save_req.len : sizeof(lora_tx_static_buf) - 1;
    for (size_t i = 0; i < tx_len; i++)
        lora_tx_static_buf[i] = save_req.data[i];
    lora_tx_static_buf[tx_len] = '\0';
    lora_req.lora_tx_len = tx_len;

    // send to queue
    xQueueSend(get_lora_queue(), &lora_req, portMAX_DELAY);
    
}
// FREE RTOS TASK

void ping_task(void *arg)
{
    // message structs
    PingParser parser;
    uart_transaction_t trans;
    ping_distance_t ping_distance_response;
    ping_speed_of_sound_t ping_speed_of_sound_response;
    ping_ack_t ack;
    ping_nack_t nack;
    ping_device_info_t device_info;
    ping_profile_t profile;
    ping_range_t ping_range_response;
    ping_mode_auto_t ping_mode_auto_response;

    // toggle for testing in a loop
    bool flip = false;
    
    // send request
    // ask for device_info on init
    send_general_request(4, &trans);   // device_info

    if (trans.rx_len <= 0) {
        ESP_LOGE(TAG, "No response during init");
    } else {
        ESP_LOGI(TAG, "Ping1D responded (%d bytes)", trans.rx_len);
    }
    
    send_set_mode_auto(0, &trans);
    set_speed_of_sound(343000, &trans); // 343 m/s at ~20°C
    set_range(100, 3000, &trans);
    set_gain_setting(6, &trans);
    vTaskDelay(pdMS_TO_TICKS(50));
   
    bool profile_made = false;
    while (1)
    {
        // Send Request

        //send_general_request(1212, &trans);   // distance
        // if (flip){
        //     //ping_distance(&trans);
        //     //get_range(&trans);
        //     //set_range(0, 2000, &trans); // scan_start=1 m, scan_length=2 m
        //     set_gain_setting(6, &trans);
        //     flip = !flip;
        // }
        // else{
        //     //set_speed_of_sound(340.0f, &trans);
        //     //send_general_request(4, &trans);
        //     //get_device_info(&trans);
        //     //send_profile_request(&trans, 0);
        //     //set_range(0, 2000, &trans); // scan_start=1 m, scan_length=2 m
        //     //get_mode_auto(&trans);
        //     //ESP_LOGI(TAG, "Requested speed of sound");
        //     send_profile_request(&trans, 0);
        //     flip = !flip;
        // }
        send_profile_request(&trans, 0);
        profile_made = false;
        // Parse Response
        if (trans.rx_len > 0)
        {
            parser.reset();

            for (int i = 0; i < trans.rx_len; i++)
            {
                auto state = parser.parseByte(trans.rx_buf[i]);

                // check for new message
                if (state == PingParser::State::NEW_MESSAGE)
                {
                    uint16_t msg_id = parser.rxMessage.message_id();
                    uint8_t *p = parser.rxMessage.payload_data();

                    // check message ID and send payload to correct parser
                    // bad implementation but it works for now
                    if (msg_id == 1) {   // ACK
                        parse_ack(p, parser.rxMessage.payload_length(), &ack);
                    }
                    else if (msg_id == 2) {   // NACK
                        parse_nack(p, parser.rxMessage.payload_length(), &nack);
                    }
                    else if (msg_id == 4) {  // Device Info
                        parse_device_info(p, parser.rxMessage.payload_length(), &device_info);
                    }
                    else if (msg_id == 1212) // Distance response
                    {
                        parse_distance(p, parser.rxMessage.payload_length(), &ping_distance_response);
                    }
                    else if (msg_id == 1002)  // Speed-of-sound response
                    {
                        parse_speed_of_sound(p, parser.rxMessage.payload_length(), &ping_speed_of_sound_response);
                    }
                    else if (msg_id == 1204) // range response
                    { 
                        parse_range(p, parser.rxMessage.payload_length(), &ping_range_response);
                    }
                    else if (msg_id == 1205 || msg_id == 1003) // mode_auto response or echo from set_mode_auto
                    {
                        parse_mode_auto(p, parser.rxMessage.payload_length(), &ping_mode_auto_response);
                    }
                    else if (msg_id == 1300) // profile response
                    {
                        parse_profile(p, parser.rxMessage.payload_length(), &profile);
                        profile_made = true;
                        // char* csvln;
                        // size_t csvlen;
                        // make_csv(&profile, &csvln, &csvlen);
                        // record_data(csvln);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "No response");
                    }
                }
                    
            }
        }
        if (profile_made)
        {
            char* csvln;
            size_t csvlen;
            make_csv(&profile, &csvln, &csvlen);
            record_data(csvln);
        }

        vTaskDelay(pdMS_TO_TICKS(g_sample_interval_ms));
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
        1
    );
}