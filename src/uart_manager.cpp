#include "uart_manager.h"
#include "hardware.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ping_task.h"


static QueueHandle_t uart_queue;
static const char *TAG = "UART_MGR";

static void mux_select(mux_device_t dev)
{
    gpio_set_level(MUX_A0, (dev >> 0) & 1);
    gpio_set_level(MUX_A1, (dev >> 1) & 1);
    gpio_set_level(MUX_A2, (dev >> 2) & 1);
}

static void debug_tx_tx(const uart_transaction_t *trans)
{
        for (int i = 0; i < trans->tx_len; i++) {
            ESP_LOGI(TAG, "tX[%d]: 0x%02X", i, trans->tx_buf[i]);
        }
        for (int i = 0; i < trans->rx_len; i++) {
            ESP_LOGI(TAG, "RX[%d]: 0x%02X", i, trans->rx_buf[i]);
        }
}
int parse_ublox_latlon_stream(const uint8_t *buf, size_t len, double *lat, double *lon) {
    int found = 0;

    for (size_t i = 0; i + 36 <= len; i++) { // need full header + payload + checksum
        if (buf[i] == 0xB5 && buf[i+1] == 0x62) {
            uint8_t cls = buf[i+2];
            uint8_t id  = buf[i+3];
            uint16_t payload_len = buf[i+4] | (buf[i+5] << 8);

            if (cls == 0x01 && id == 0x02 && payload_len >= 28) {
                // simple checksum check (optional)
                uint8_t ck_a = 0, ck_b = 0;
                for (size_t j = 2; j < 6 + payload_len; j++) {
                    ck_a += buf[i+j];
                    ck_b += ck_a;
                }
                if (ck_a != buf[i+6+payload_len] || ck_b != buf[i+6+payload_len+1])
                    continue; // bad checksum, skip

                // Extract lat/lon
                int32_t raw_lon = buf[i+6+4] | (buf[i+6+5]<<8) | (buf[i+6+6]<<16) | (buf[i+6+7]<<24);
                int32_t raw_lat = buf[i+6+8] | (buf[i+6+9]<<8) | (buf[i+6+10]<<16) | (buf[i+6+11]<<24);

                *lon = raw_lon * 1e-7;
                *lat = raw_lat * 1e-7;
                found = 1; // keep scanning for last message
            }
        }
    }

    return found;
}

static void uart_manager_task(void *arg)
{
    uart_transaction_t *trans;
    while (1)
    {

        if (xQueueReceive(uart_queue, &trans, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Writing to device %d", trans->device);
            mux_select(trans->device);
            vTaskDelay(pdMS_TO_TICKS(10));
            uart_flush(UART_PORT);

            uart_write_bytes(UART_PORT,
                             (const char*)trans->tx_buf,
                             trans->tx_len);

            //uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));
            
            int len = uart_read_bytes(UART_PORT,
                                      trans->rx_buf,
                                      512,
                                      pdMS_TO_TICKS(500));    
            trans->rx_len = len;

            ESP_LOGI(TAG, "DEV %d TX: %d bytes, RX: %d bytes", trans->device, trans->tx_len, len);
            // debug_tx_tx(trans);



            if (trans->caller)
                xTaskNotifyGive(trans->caller);

        }
    }
}

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
    const int UART_BUF_SIZE = 2048;
    // ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE, 10, NULL, 0));
    ESP_ERROR_CHECK(uart_driver_install(
                    UART_PORT,
                    UART_BUF_SIZE,   // RX buffer
                    0,      // TX buffer disabled
                    0,      // no event queue
                    NULL,
                    0));

    uart_config_t uart_config = {
            //.baud_rate = 230400,
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