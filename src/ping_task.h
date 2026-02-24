#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

// Ping distance format:
    // u32 distance_mm
    // u16 confidence  %
    // u16 transmit_duration  us
    // u32 ping_number
    // u32 scan_start  mm
    // u32 scan_length  mm 
    // u32 gain_setting
typedef struct {
    uint32_t distance_mm;
    uint16_t confidence;
    uint16_t transmit_duration_us;
    uint32_t ping_number;
    uint32_t scan_start_mm;
    uint32_t scan_length_mm;
    uint32_t gain_setting;
    uint32_t timestamp;
} ping_distance_t;

QueueHandle_t get_ping_queue();
void init_ping_task();