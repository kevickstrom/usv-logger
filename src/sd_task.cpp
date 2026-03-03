#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/gpio.h"
#include "sd_task.h"
#include "hardware.h"
#include "aggregator.h"
#include "config.h"
#include "esp_timer.h"

#define SD_BUFFER_SIZE 4096
#define MOUNT_POINT "/sdcard"

static char sd_buffer[SD_BUFFER_SIZE];
static const char *TAG = "SD_TASK";
static QueueHandle_t save_queue;


QueueHandle_t get_save_queue()
{
    return save_queue;
}

static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, "%s", data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[512];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

static int handle_save(save_req_t *save_req)
{
    esp_err_t good;
    ESP_LOGI(TAG, "GOT FNAME: %s", save_req->fname);
    char path[64];
    snprintf(path, sizeof(path), "%s/%s", MOUNT_POINT, save_req->fname);
    good = s_example_write_file(path, save_req->data);
    if (good != ESP_OK)
    {
        return 1;
    }
    return 0;
}

static void sd_task(void *arg)
{
    esp_err_t ret;
    save_req_t save_req;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
    .mosi_io_num = SPI_MOSI,
    .miso_io_num = SPI_MISO,
    .sclk_io_num = SPI_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
    };

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI_CS0;
    //slot_config.host_id = host.slot;
    slot_config.host_id = SPI2_HOST;

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus (%s)", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) 
    {
        if (ret == ESP_FAIL) 
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                        "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } 
        else 
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                        "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
            }
            return;
        }
    ESP_LOGI(TAG, "Filesystem mounted");

    sdmmc_card_print_info(stdout, card);

    const char *file_hello = MOUNT_POINT"/hello.txt";
    char data[512];
    snprintf(data, 512, "%s %s!\n", "Hello", card->cid.name);
    ret = s_example_write_file(file_hello, data);
    if (ret != ESP_OK) {
        return;
    }
        
    TickType_t last_wake = xTaskGetTickCount();
    int pin_level = gpio_get_level(TOGGLE_SW);
    while (1)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(g_log_interval_ms));
        pin_level = gpio_get_level(TOGGLE_SW);
        ESP_LOGI(TAG, "LVL: %d", pin_level);
        while (xQueueReceive(get_save_queue(), &save_req, 0))
        {
            if (pin_level)
            {
                handle_save(&save_req);
                ESP_LOGI(TAG, "SAVED");
            }
            else
            {
                ESP_LOGI(TAG, "Logging currently disabled");

            }
        }
    }
}


void init_sd_task()
{
    gpio_set_direction(SPI_CLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_MISO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_CS0, GPIO_MODE_OUTPUT);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOGGLE_SW), // Select the pin
        .mode = GPIO_MODE_INPUT,               // Set as input mode
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Enable internal pull-up resistor (optional, e.g., for a button connected to GND)
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable internal pull-down resistor
        .intr_type = GPIO_INTR_DISABLE         // Disable interrupts
    };
    gpio_config(&io_conf);

    save_queue = xQueueCreate(10, sizeof(save_req_t));
    xTaskCreatePinnedToCore(
        sd_task,
        "sd_task",
        4*4096,
        NULL,
        4,
        NULL,
        1
    );
}