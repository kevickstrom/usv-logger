#include "ping_task.h"
#include "uart_manager.h"
#include "config.h"
#include "esp_timer.h"
#include <string.h>

static QueueHandle_t ping_queue;

QueueHandle_t get_ping_queue()
{
    return ping_queue;
}

static void ping_task(void *arg)
{
    uart_transaction_t trans;
    ping_data_t ping;

    while (1)
    {
        // Example Ping command (replace with real protocol)
        const char *cmd = "PING_MEASURE\n";

        memset(&trans, 0, sizeof(trans));
        trans.device = PING;
        memcpy(trans.tx_buf, cmd, strlen(cmd));
        trans.tx_len = strlen(cmd);
        trans.timeout_ms = 200;
        trans.caller = xTaskGetCurrentTaskHandle();

        xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // TODO: Parse binary Ping2 response properly
        // Dummy placeholder:
        ping.depth = 10.0f;
        ping.timestamp = esp_timer_get_time() / 1000;

        xQueueSend(ping_queue, &ping, 0);

        vTaskDelay(pdMS_TO_TICKS(g_sample_interval_ms));
    }
}

void init_ping_task()
{
    ping_queue = xQueueCreate(5, sizeof(ping_data_t));

    xTaskCreatePinnedToCore(
        ping_task,
        "ping",
        4096,
        NULL,
        8,
        NULL,
        0
    );
}