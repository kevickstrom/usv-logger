#include "uart_manager.h"
#include "hardware.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ping_task.h"
#include "ping-message.h"
#include "ping-parser.h"




static QueueHandle_t uart_queue;
static const char *TAG = "UART_MGR";

static void mux_select(mux_device_t dev)
{
    gpio_set_level(MUX_A0, (dev >> 0) & 1);
    gpio_set_level(MUX_A1, (dev >> 1) & 1);
    gpio_set_level(MUX_A2, (dev >> 2) & 1);
}
static void send_general_request(uint16_t requested_id)
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

    uart_write_bytes(UART_PORT,
                     (const char*)msg.msgData,
                     msg.msgDataLength());
}
static void set_speed_of_sound(float sos_m_s)
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

    uart_write_bytes(UART_PORT,
                     (const char*)msg.msgData,
                     msg.msgDataLength());

    ESP_LOGI(TAG, "Set speed of sound to %.2f m/s", sos_m_s);
}

static void uart_manager_task(void *arg)
{
    uart_transaction_t *trans;
    uint8_t rx_buf[512];
    while (1)
    {
        ESP_LOGI(TAG, "RDY");
        send_general_request(1);  
        int len = uart_read_bytes(UART_PORT,
                              rx_buf,
                              sizeof(rx_buf),
                              pdMS_TO_TICKS(500));
        set_speed_of_sound(343.0f); // 343 m/s at ~20°C
        vTaskDelay(pdMS_TO_TICKS(50));
        if (xQueueReceive(uart_queue, &trans, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Writing to device %d", trans->device);
            mux_select(trans->device);
            vTaskDelay(pdMS_TO_TICKS(10));
            //uart_flush(UART_PORT);
            //for (int i = 0; i < trans->tx_len; i++) {
             //   ESP_LOGI(TAG, "TX[%d]: 0x%02X", i, trans->tx_buf[i]);
            //}
            //ESP_LOGI(TAG, "sz[ %d ]", trans->rx_len);
            //trans->rx_len = 512;


            ESP_LOGI(TAG, "new sz[ %d ]", trans->rx_len);
            uart_write_bytes(UART_PORT,
                             (const char*)trans->tx_buf,
                             trans->tx_len);

            //uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));
            
            //size_t length = 0;
            //ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT, (size_t*)&length));
            //send_general_request(1212); 
            int len = uart_read_bytes(UART_PORT,
                                      rx_buf,
                                      sizeof(rx_buf),
                                      pdMS_TO_TICKS(500));

            //trans->rx_len = len;
            ESP_LOGI(TAG, "read [ %d ]", len);
            

/*
            size_t total_received = 0;
            uint16_t expected_len = 0;
            TickType_t start = xTaskGetTickCount();

            while (xTaskGetTickCount() - start < pdMS_TO_TICKS(trans->timeout_ms)) {
                int len = uart_read_bytes(UART_PORT,
                                        trans->rx_buf + total_received,
                                        sizeof(trans->rx_buf) - total_received,
                                        pdMS_TO_TICKS(10));
                if (len > 0) {
                    total_received += len;
                    if (total_received >= 6) { // can read payload length from header
                        expected_len = 1+1+2+2+1+1 + (trans->rx_buf[2] | (trans->rx_buf[3]<<8)) + 2;
                        if (total_received >= expected_len) break;
                    }
                }
            }
            trans->rx_len = total_received;
*/

            if (trans->caller)
                xTaskNotifyGive(trans->caller);

            ESP_LOGI(TAG, "DEV %d TX: %d bytes, RX: %d bytes", trans->device, trans->tx_len, len);
            for (int i = 0; i < len; i++) {
                ESP_LOGI(TAG, "aaaRX[%d]: 0x%02X", i, rx_buf[i]);
            }
        }
    }
}


/*
static void uart_manager_task(void *arg)
{
    uint8_t rx_buf[512];
    PingParser parser;

    ESP_LOGI(TAG, "Syncing with Ping1D...");

    // ---- Step 1: Request device info ----
    send_general_request(1);  // device_information
    int len = uart_read_bytes(UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(500));
    if (len <= 0) {
        ESP_LOGE(TAG, "No response during init");
    } else {
        ESP_LOGI(TAG, "Ping1D responded (%d bytes)", len);
    }

    // ---- Set speed of sound ----
    set_speed_of_sound(343.0f);
    vTaskDelay(pdMS_TO_TICKS(50));

    // ---- Main loop: read distance ----
    while (1)
    {
        // Send distance request
        send_general_request(1212);  // distance

        // Read response
        len = uart_read_bytes(UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(500));
        if (len > 0)
        {
            parser.reset();
            for (int i = 0; i < len; i++)
            {
                if (parser.parseByte(rx_buf[i]) == PingParser::State::NEW_MESSAGE)
                {
                    uint16_t msg_id = parser.rxMessage.message_id();
                    if (msg_id == 1212)
                    {
                        uint8_t *p = parser.rxMessage.payload_data();
                        uint16_t distance_mm = p[0] | (p[1] << 8);
                        uint8_t confidence = p[2];
                        ESP_LOGI(TAG, "Distance: %.3f m, Confidence: %d", distance_mm / 1000.0f, confidence);
                    }
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "No response from Ping1D");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
*/
QueueHandle_t get_uart_queue()
{
    return uart_queue;
}

void init_uart_manager()
{
    gpio_set_direction(MUX_A0, GPIO_MODE_OUTPUT);
    gpio_set_direction(MUX_A1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MUX_A2, GPIO_MODE_OUTPUT);
    uart_queue = xQueueCreate(10, sizeof(uart_transaction_t*));
    mux_select(PING);
    const int UART_BUF_SIZE = 1024;
    // ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE, 10, NULL, 0));
    ESP_ERROR_CHECK(uart_driver_install(
    UART_PORT,
    2048,   // RX buffer
    0,      // TX buffer disabled
    0,      // no event queue
    NULL,
    0));
    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreatePinnedToCore(
        uart_manager_task,
        "uart_mgr",
        4096,
        NULL,
        10,
        NULL,
        0
    );
}