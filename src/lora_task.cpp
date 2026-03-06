#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "config.h"
#include "uart_manager.h"
#include "lora_task.h"
#include "rn2483.h"


static const char *TAG = "LORA";
static int currBaud = 57600;

static QueueHandle_t lora_queue;
static lora_request_t lora_req;
static uart_transaction_t trans;

// Forward declaration of the task
static void lora_task(void *arg);

// --- Public API ---
void init_lora_task()
{
    
    lora_queue = xQueueCreate(10, sizeof(lora_request_t));

    // Create FreeRTOS task pinned to core 1 (aggregator/core separation)
    xTaskCreatePinnedToCore(
        lora_task,
        "LORA_TASK",
        4096,   // stack size
        NULL,   // task parameter
        5,      // priority
        NULL,   // task handle
        1       // core
    );
}

QueueHandle_t get_lora_queue()
{
    return lora_queue;
}

lora_request_t* get_curr_request()
{
    return &lora_req;
}

void write_uart(const char* data, size_t len)
{
    ESP_LOGI(TAG, "building trans");
    memset(&trans, 0, sizeof(trans));

    memcpy(trans.tx_buf, data, len);
    trans.tx_len = len;
    trans.device = LORA;
    trans.baud = currBaud;
    trans.timeout_ms = 500;
    trans.caller = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "built trans");
    uart_transaction_t *ptr = &trans;
    if(xQueueSend(get_uart_queue(), &ptr, portMAX_DELAY) != pdTRUE)
    {
        return;
    }
    ESP_LOGI(TAG, "put on queue");

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    size_t copy_len = trans.rx_len;
    if (copy_len >= sizeof(lora_req.lora_rx_buf))
        copy_len = sizeof(lora_req.lora_rx_buf) - 1;

    memcpy(lora_req.lora_rx_buf, trans.rx_buf, copy_len);
    lora_req.lora_rx_buf[copy_len] = '\0';
    lora_req.lora_rx_len = copy_len;
}

// set gpio led on RN
void set_led1()
{
    char response[64];
    ESP_LOGI(TAG, "SET LED");
    RN2483_command("sys set pindig GPIO8 1\r\n", response);

}

// RN Radio init params
void lora_param_init()
{
    char response[64];
    RN2483_command("mac pause\r\n", response);
    ESP_LOGI(TAG, "MAC paused: %s", response);
    // Set LoRa radio mode: SF7BW125, 868 MHz (example for EU868)
    ESP_LOGI(TAG, "set mod lora");
    RN2483_command("radio set mod lora\r\n", response);
    ESP_LOGI(TAG, "RN: %c", response);

    ESP_LOGI(TAG, "set freq");
    // Set frequency (example 868.1 MHz)
    RN2483_command("radio set freq 868100000\r\n", response);
    ESP_LOGI(TAG, "RN: %c", response);

    ESP_LOGI(TAG, "Set sf");
    // Set spreading factor (SF7-SF12)
    RN2483_command("radio set sf sf7\r\n", response);
    ESP_LOGI(TAG, "RN: %c", response);

    ESP_LOGI(TAG, "set bandwidth");
    // Set bandwidth (BW125 = 125 kHz)
    RN2483_command("radio set bw 125\r\n", response);
    ESP_LOGI(TAG, "RN: %c", response);

    ESP_LOGI(TAG, "Set coding rate");
    // Set coding rate (4/5 default)
    RN2483_command("radio set cr 4/5\r\n", response);
    ESP_LOGI(TAG, "RN: %c", response);

    ESP_LOGI(TAG, "Set Power 14");
    // Optional: set power (2–14 dBm typical)
    RN2483_command("radio set pwr 14\r\n", response);
    ESP_LOGI(TAG, "RN: %c", response);
}

// Convert a string to hex representation
void str_to_hex(const char* input, char* output)
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

void radio_tx(const char* input, size_t len)
{
    char payload_hex[2*len + 1]; // 2 hex chars per byte + null terminator
    str_to_hex(input, payload_hex); // convert your input to hex

    char cmd[2*len + 20]; // enough for "radio tx " + payload + "\r\n"
    snprintf(cmd, sizeof(cmd), "radio tx %s\r\n", payload_hex);

    char response[64];
    RN2483_command(cmd, response);
    ESP_LOGI(TAG, "RN: %c", response);
}

// reset gpio led
void reset_led1()
{
    char response[64];
    RN2483_command("sys set pindig GPIO8 0\r\n", response);
    //lora_param_init();
}


// --- Private task ---
static void lora_task(void *arg)
{

    // empty req
    lora_req.id = -1;
    bool toggled = false;
    TickType_t last_wake = xTaskGetTickCount();
    ESP_LOGI(TAG, "almost rdy");
    while (1)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(g_sample_interval_ms));
        TickType_t now = xTaskGetTickCount();
        if ((now - last_wake) >= pdMS_TO_TICKS(g_log_interval_ms))
        {
            if (toggled)
            {
                ESP_LOGI(TAG, "goingto");
                reset_led1();
                toggled = !toggled;
            }
            else
            {
                ESP_LOGI(TAG, "gunna");
                set_led1();
                toggled = !toggled;
            }
        }

        char msg[] = "hello";
        radio_tx(msg, strlen(msg));
        ESP_LOGI(TAG, "TX: %s", msg);

    }
}