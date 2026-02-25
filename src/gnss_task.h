#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

typedef struct {
    double lat;
    double lon;
    float alt_m;
    uint8_t fix_type; // 0 = no fix, 1 = 2D fix, 2 = 3D fix
    uint8_t num_sats;
    uint32_t timestamp;
} gps_data_t;

QueueHandle_t get_gps_queue();
void init_gnss_task();