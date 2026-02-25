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
            //debug_tx_tx(trans);

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