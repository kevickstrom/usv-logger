#include "sd_task.h"
#include "aggregator.h"
#include "config.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

#define SD_BUFFER_SIZE 4096

static char sd_buffer[SD_BUFFER_SIZE];
static size_t buffer_index = 0;
static uint64_t last_flush_time = 0;

static void flush_to_sd()
{
    // TODO: Implement real FATFS write
    // fwrite(sd_buffer, 1, buffer_index, file);

    buffer_index = 0;
    last_flush_time = esp_timer_get_time() / 1000;
}

static void sd_task(void *arg)
{
    record_t rec;
    last_flush_time = esp_timer_get_time() / 1000;

    while (1)
    {
        if (xQueueReceive(get_record_queue(), &rec, portMAX_DELAY))
        {
            char line[128];

            int len = snprintf(line, sizeof(line),
                               "%lu,%.6f,%.6f,%.2f\n",
                               rec.timestamp,
                               rec.lat,
                               rec.lon,
                               rec.depth);

            if (buffer_index + len < SD_BUFFER_SIZE)
            {
                memcpy(&sd_buffer[buffer_index], line, len);
                buffer_index += len;
            }

            uint64_t now = esp_timer_get_time() / 1000;

            if ((now - last_flush_time) >= g_log_interval_ms)
            {
                flush_to_sd();
            }
        }
    }
}

void init_sd_task()
{
    xTaskCreatePinnedToCore(
        sd_task,
        "sd_task",
        4096,
        NULL,
        4,
        NULL,
        1
    );
}