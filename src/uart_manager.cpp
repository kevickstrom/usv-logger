#include "uart_manager.h"
#include "hardware.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ping_task.h"

#define DEFAULT_BAUD 115200

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

static void hex_to_ascii(const char* hex, char* ascii, size_t max_len) {
    size_t i = 0;
    while (*hex && i < max_len - 1) {
        unsigned int val;
        if (sscanf(hex, "%2x", &val) == 1) {
            ascii[i++] = (char)val;
            hex += 2;
        } else {
            break;
        }
    }
    ascii[i] = '\0';
}

// Convert a string to hex representation
static void str_to_hex(const char* input, char* output)
{
    const char hex_chars[] = "0123456789ABCDEF";
    while (*input)
    {
        uint8_t c = *input++;
        *output++ = hex_chars[(c >> 4) & 0x0F];
        *output++ = hex_chars[c & 0x0F];
    }
    *output = '\0'; // null terminate
}

void log_rn2483_transaction(uart_transaction_t* trans)
{
    static const char *TAG = "RN2483";

    // --- TX ---
    ESP_LOGI(TAG, "TX (%d bytes):", trans->tx_len);

    char tx_hex[trans->tx_len * 2 + 1]; // each byte -> 2 hex chars
    for (int i = 0; i < trans->tx_len; i++) {
        uint8_t c = trans->tx_buf[i];
        if (c >= 32 && c <= 126) {
            // printable ASCII, keep as-is
            tx_hex[i] = c;
        } else {
            // non-printable: convert to hex
            char tmp[3];
            sprintf(tmp, "%02X", c);
            tx_hex[i*2] = tmp[0];
            tx_hex[i*2+1] = tmp[1];
        }
    }
    tx_hex[trans->tx_len * 2] = '\0';
    ESP_LOGI(TAG, "%s", tx_hex);

    // --- RX ---
    ESP_LOGI(TAG, "RX (%d bytes):", trans->rx_len);

    char rx_ascii[128];
    char rx_hex[trans->rx_len * 2 + 1];

    // Copy RX buffer to a hex string using str_to_hex
    memcpy(rx_hex, trans->rx_buf, trans->rx_len);
    rx_hex[trans->rx_len] = '\0';
    str_to_hex((char*)trans->rx_buf, rx_hex);

    // Convert back to ASCII for logging (optional)
    hex_to_ascii(rx_hex, rx_ascii, sizeof(rx_ascii));

    ESP_LOGI(TAG, "%s", rx_ascii);
}

static void uart_manager_task(void *arg)
{
    uart_transaction_t *trans;
    int currBaud = DEFAULT_BAUD;
    while (1)
    {

        if (xQueueReceive(uart_queue, &trans, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Writing to device %d", trans->device);
            mux_select(trans->device);
            vTaskDelay(pdMS_TO_TICKS(10));
            uart_flush(UART_PORT);

            // if device wants a different baudrate
            if (trans->baud != currBaud)
            {
                uart_set_baudrate(UART_PORT, trans->baud);
                currBaud = trans->baud;
            }

            uart_write_bytes(UART_PORT,
                             (const char*)trans->tx_buf,
                             trans->tx_len);

            //uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));
            
            int len = uart_read_bytes(UART_PORT,
                                      trans->rx_buf,
                                      512,
                                      pdMS_TO_TICKS(trans->timeout_ms));    
            trans->rx_len = len;

            ESP_LOGI(TAG, "DEV %d TX: %d bytes, RX: %d bytes", trans->device, trans->tx_len, len);
            //debug_tx_tx(trans);
            //log_rn2483_transaction(trans);



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
    // TODO: Handle default baud rates and baud rate switching
    gpio_set_direction(MUX_A0, GPIO_MODE_OUTPUT);
    gpio_set_direction(MUX_A1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MUX_A2, GPIO_MODE_OUTPUT);

    uart_queue = xQueueCreate(40, sizeof(uart_transaction_t*));

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
            //.baud_rate = 115200,
            .baud_rate = DEFAULT_BAUD,
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