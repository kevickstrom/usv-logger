#include "gnss_task.h"
#include "uart_manager.h"
#include "config.h"
#include <string.h>

static QueueHandle_t gps_queue;

static void gnss_task(void *arg)
{
    uart_transaction_t trans;

    while (1)
    {
        memset(&trans, 0, sizeof(trans));
        trans.device = GPS;
        trans.timeout_ms = 100;
        trans.caller = xTaskGetCurrentTaskHandle();

        xQueueSend(get_uart_queue(), &trans, portMAX_DELAY);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // TODO: Parse GNSS data

        vTaskDelay(pdMS_TO_TICKS(g_sample_interval_ms));
    }
}

QueueHandle_t get_gps_queue()
{
    return gps_queue;
}

void init_gnss_task()
{
    gps_queue = xQueueCreate(5, sizeof(gps_data_t));

    xTaskCreatePinnedToCore(
        gnss_task,
        "gnss",
        4096,
        NULL,
        9,
        NULL,
        0
    );
}