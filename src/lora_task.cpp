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
char response[128];

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
    memset(&trans, 0, sizeof(trans));

    memcpy(trans.tx_buf, data, len);
    trans.tx_len = len;
    trans.device = LORA;
    trans.baud = currBaud;
    trans.timeout_ms = 500;
    trans.caller = xTaskGetCurrentTaskHandle();
    uart_transaction_t *ptr = &trans;
    if(xQueueSend(get_uart_queue(), &ptr, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

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
    RN2483_command("sys set pindig GPIO8 1\r\n", response);

}

// RN Radio init params
void lora_param_init() {
    ESP_LOGI(TAG, "Pausing MAC");
    RN2483_command("mac pause\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Setting LoRa mode");
    RN2483_command("radio set mod lora\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Setting frequency");
    RN2483_command("radio set freq 865000000\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Setting bandwidth");
    RN2483_command("radio set bw 125\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Setting spreading factor");
    RN2483_command("radio set sf sf7\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Setting coding rate");
    RN2483_command("radio set cr 4/5\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Setting preamble length");
    RN2483_command("radio set prlen 8\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Enable CRC");
    RN2483_command("radio set crc on\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Set sync word");
    RN2483_command("radio set sync 12\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Set TX power");
    RN2483_command("radio set pwr 14\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);

    ESP_LOGI(TAG, "Disable watchdog");
    RN2483_command("radio set wdt 0\r\n", response);
    ESP_LOGI(TAG, "RN: %s", response);
}

void hex_to_ascii(const char* hex, char* ascii, size_t max_len) {
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

    //const char* cmd = "radio tx 48656C6C6F\r\n";

    ESP_LOGI(TAG, "req: %s", cmd);
    RN2483_command(cmd, response);
    // ESP_LOGI(TAG, "RN: %s", response);
    // if (strcmp(response, "ok\r\n") == 0)
    //     {
    //         RN2483_response((uint8_t*)response);
    //         ESP_LOGI(TAG, "TX result: %s", response);
    //     }
    
}

// reset gpio led
void reset_led1()
{
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
    lora_param_init();
    while (1)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(g_sample_interval_ms));
        // TickType_t now = xTaskGetTickCount();
        //         const char* msg = "Hello";
        // radio_tx(msg, 6);
        
        while (xQueueReceive(get_lora_queue(), &lora_req, 0))
        {
            
            radio_tx(lora_req.lora_tx_buf, lora_req.lora_tx_len);
        }

        // if ((now - last_wake) >= pdMS_TO_TICKS(g_log_interval_ms))
        // {
        //     if (toggled)
        //     {
        //         reset_led1();
        //         toggled = !toggled;
        //     }
        //     else
        //     {
        //         set_led1();
        //         toggled = !toggled;
        //     }
        // }

        //char msg[] = "hello";

        //radio_tx();
        //ESP_LOGI(TAG, "TX: %s", msg);
        // RN2483_command("radio tx 48656C6C6F\r\n", response);
        // ESP_LOGI(TAG, "TX start: %s", response);

        // if (strcmp(response, "ok\r\n") == 0)
        // {
        //     RN2483_response((uint8_t*)response);
        //     ESP_LOGI(TAG, "TX result: %s", response);
        // }

    }
}