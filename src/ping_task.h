#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

typedef struct {
    float depth_m;
    uint8_t confidence;
    uint32_t timestamp;
} ping_data_t;

QueueHandle_t get_ping_queue();
void init_ping_task();