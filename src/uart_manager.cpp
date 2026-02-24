#include "uart_manager.h"
#include "hardware.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"




static QueueHandle_t uart_queue;
static const char *TAG = "UART_MGR";

static void mux_select(mux_device_t dev)
{
    gpio_set_level(MUX_A0, (dev >> 0) & 1);
    gpio_set_level(MUX_A1, (dev >> 1) & 1);
    gpio_set_level(MUX_A2, (dev >> 2) & 1);
}

static void uart_manager_task(void *arg)
{
    uart_transaction_t trans;

    while (1)
    {
        if (xQueueReceive(uart_queue, &trans, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Writing to device %d", trans.device);
            mux_select(trans.device);

            uart_flush(UART_PORT);
            uart_write_bytes(UART_PORT,
                             (const char*)trans.tx_buf,
                             trans.tx_len);

            int len = uart_read_bytes(UART_PORT,
                                      trans.rx_buf,
                                      sizeof(trans.rx_buf),
                                      pdMS_TO_TICKS(trans.timeout_ms));

            trans.rx_len = len;

            if (trans.caller)
                xTaskNotifyGive(trans.caller);
        }
    }
}

QueueHandle_t get_uart_queue()
{
    return uart_queue;
}

void init_uart_manager()
{
    uart_queue = xQueueCreate(10, sizeof(uart_transaction_t));

    const int UART_BUF_SIZE = 1024;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_1, &uart_config);

    // Install the driver with RX/TX buffers and event queue (pass uart_queue if you want task notifications)
    uart_driver_install(UART_NUM_1, UART_BUF_SIZE, UART_BUF_SIZE, 10, NULL, 0);

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