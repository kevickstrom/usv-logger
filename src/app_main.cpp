#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "gnss_task.h"
#include "ping_task.h"
#include "aggregator.h"
#include "sd_task.h"
#include "lora_task.h"
#include "uart_manager.h"
#include "ping_test.h"

static const char *TAG = "MAIN";

// Correct extern "C" for app_main only
extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting USV logger system...");

    // Set global intervals
    g_sample_interval_ms = 500;
    g_log_interval_ms = 2000;

    // Initialize tasks
    ESP_LOGI(TAG, "Initializing UART task...");
    init_uart_manager();

    ESP_LOGI(TAG, "Initializing GNSS task...");
    //init_gnss_task();

    ESP_LOGI(TAG, "Initializing Ping task...");
    init_ping_task();
    //init_ping_distance_task();

    ESP_LOGI(TAG, "Initializing Aggregator...");
    //init_aggregator();

    ESP_LOGI(TAG, "Initializing SD task...");
    //init_sd_task();

    ESP_LOGI(TAG, "Initializing LoRa task...");
    //init_lora_task();

    ESP_LOGI(TAG, "All tasks started. System running.");

    // Main loop
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "System alive...");
    }
}