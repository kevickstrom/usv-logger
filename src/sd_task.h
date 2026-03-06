#pragma once
#include "uart_manager.h"

#define MAX_FNAME 32
#define MAX_DATA  1024

#define MAX_OPEN_FILES 5
#define LOG_BUFFER_SIZE 4096
#define LOG_FLUSH_INTERVAL_MS 1000
#define MAX_FILE_HOLD_TIME_MS 1000

#define SD_BUFFER_SIZE 4096
#define MOUNT_POINT "/sdcard"

typedef struct {
    char fname[MAX_FNAME];
    int device;
    char data[MAX_DATA];
    uint32_t len;
} save_req_t;

typedef struct {
    char fname[MAX_FNAME];
    FILE *fp;
    char buffer[LOG_BUFFER_SIZE];
    size_t index;
    TickType_t last_flush_tick;
    TickType_t last_write_tick;
} file_log_t;

QueueHandle_t get_save_queue();
void init_sd_task();