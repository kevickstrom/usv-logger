#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

typedef struct {
    double lat;
    double lon;
    float depth;
    uint32_t timestamp;
} record_t;

QueueHandle_t get_record_queue();
void init_aggregator();