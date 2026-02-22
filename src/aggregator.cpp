#include "aggregator.h"
#include "gnss_task.h"
#include "ping_task.h"
#include <string.h>

static QueueHandle_t record_queue;

QueueHandle_t get_record_queue()
{
    return record_queue;
}

static void aggregator_task(void *arg)
{
    gps_data_t gps;
    ping_data_t ping;
    record_t rec;

    while (1)
    {
        if (xQueueReceive(get_gps_queue(), &gps, portMAX_DELAY))
        {
            rec.lat = gps.lat;
            rec.lon = gps.lon;
            rec.timestamp = gps.timestamp;

            // Try to match latest ping
            if (xQueueReceive(get_ping_queue(), &ping, pdMS_TO_TICKS(200)))
            {
                rec.depth = ping.depth;
            }

            xQueueSend(record_queue, &rec, 0);
        }
    }
}

void init_aggregator()
{
    record_queue = xQueueCreate(10, sizeof(record_t));

    xTaskCreatePinnedToCore(
        aggregator_task,
        "aggregator",
        4096,
        NULL,
        6,
        NULL,
        1
    );
}