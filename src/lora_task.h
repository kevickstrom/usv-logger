# pragma once
#include <stdint.h>

typedef struct {
    int device;
    int id;
    char* lora_tx_buf;
    size_t lora_tx_len;
    char lora_rx_buf[64];
    size_t lora_rx_len;
}lora_request_t;

QueueHandle_t get_lora_queue();
void write_uart(const char* data, size_t len);
lora_request_t* get_curr_request();
void init_lora_task();