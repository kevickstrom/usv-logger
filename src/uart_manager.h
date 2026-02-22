#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    GPS = 0,
    PING = 1,
    FC = 2,
    LORA = 3
} mux_device_t;

typedef struct {
    mux_device_t device;
    uint8_t tx_buf[128];
    size_t tx_len;
    uint8_t rx_buf[256];
    size_t rx_len;
    uint32_t timeout_ms;
    TaskHandle_t caller;
}uart_transaction_t;

QueueHandle_t get_uart_queue();
void init_uart_manager();
