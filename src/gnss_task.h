#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

typedef struct {
    double lat;
    double lon;
    uint32_t timestamp;
} gps_data_t;

QueueHandle_t get_gps_queue();
void init_gnss_task();