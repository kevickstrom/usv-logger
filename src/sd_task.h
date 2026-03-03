#pragma once
#include "uart_manager.h"

#define MAX_FNAME 32
#define MAX_DATA  1024

typedef struct {
    char fname[MAX_FNAME];
    int device;
    char data[MAX_DATA];
    uint32_t len;
} save_req_t;

QueueHandle_t get_save_queue();
void init_sd_task();