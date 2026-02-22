#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "config.h"
#include "uart_manager.h"

static const char *TAG = "LORA";

// Queue to send messages to this task (optional)
static QueueHandle_t lora_queue;

// Forward declaration of the task
static void lora_task(void *arg);

// --- Public API ---
void init_lora_task()
{
    // Create a queue if needed (optional, e.g., for messages to send)
    lora_queue = xQueueCreate(10, sizeof(char[128])); // queue for 128-byte messages

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

// --- Private task ---
static void lora_task(void *arg)
{
    uart_transaction_t trans;
    char send_buffer[128];

    while (1)
    {
        if (xQueueReceive(lora_queue, send_buffer, portMAX_DELAY))
        {
            memset(&trans, 0, sizeof(trans));
            trans.device = LORA;
            trans.timeout_ms = 500;
            trans.caller = xTaskGetCurrentTaskHandle();

            // Copy the data into the transaction's tx buffer
            memcpy(trans.tx_buf, send_buffer, strlen(send_buffer));
            trans.tx_len = strlen(send_buffer);

            // No RX expected
            trans.rx_len = 0;

            xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(trans.timeout_ms));

            ESP_LOGI(TAG, "Sent LoRa message: %s", send_buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(g_log_interval_ms));
    }
}