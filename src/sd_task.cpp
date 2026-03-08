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

static file_log_t open_files[MAX_OPEN_FILES];
static int num_open_files = 0;

static const char *TAG = "SD_TASK";

static QueueHandle_t save_queue;

QueueHandle_t get_save_queue()
{
    return save_queue;
}

//
// Fill file array with blank structs
//
static void init_file_array()
{
    for (int i = 0; i < MAX_OPEN_FILES; i++)
    {
        file_log_t *ft = &open_files[i];
        TickType_t now = xTaskGetTickCount();
        ft->fname[0] = '\0';
        ft->index = 0;
        ft->fp = NULL;
        memset(ft->buffer, 0, LOG_BUFFER_SIZE);
        ft->last_flush_tick = now;
        ft->last_write_tick = now;
    }

}

//
// Closes a file that's currently held open in the open files array
// Basically just fills the struct with 0's
//
static void close_file(file_log_t *ft)
{

    if (ft->index > 0)
    {
        // there's still data in the buffer i need to write
        fwrite(ft->buffer, 1, ft->index, ft->fp);
        fflush(ft->fp);
    }
    
    TickType_t now = xTaskGetTickCount();
    ft->fname[0] = '\0';
    ft->index = 0;
    memset(ft->buffer, 0, sizeof(ft->buffer));
    ft->last_flush_tick = now;
    ft->last_write_tick = now;

    fclose(ft->fp);
    ft->fp = NULL;
    num_open_files -= 1;

}


//
// Returns the struct of an open file
// Handles open file array and closes oldest files if necessary
//
static file_log_t* get_or_open_file(const char *path)
{
    TickType_t now = xTaskGetTickCount();

    // check if the file is open
    for (int i = 0; i < MAX_OPEN_FILES; i++)
    {
        if (open_files[i].fname[0] != '\0' &&
            strcmp(open_files[i].fname, path) == 0)
        {
            ESP_LOGI(TAG, "Found open file");
            return &open_files[i]; // found file already open
        }
    }
    ESP_LOGI(TAG, "MAKINGFILE");
    file_log_t *file = NULL;
    // file not yet opened
    // put file in rotation
    // check if space available / make space
    if (num_open_files < MAX_OPEN_FILES)
    {
        // open space, add to list of current files and increment count
        // find first open spot in open file list
        for (int i = 0; i < MAX_OPEN_FILES; i++)
        {
            if (open_files[i].fname[0] == '\0')
            {
                file = &open_files[i];
                file->fp = fopen(path, "a");
                //snprintf(file->fname, sizeof(file->fname), "%s", path);
                //num_open_files++;
                break;
            }
        }
        // still need to fill out file_log_t so dont return
        //return file;
    }
    else
    {
        // space is full we need to drop the least used file
        int lru_index = 0;

        TickType_t oldest = open_files[0].last_write_tick;

        for (int i = 0; i < MAX_OPEN_FILES; i++)
        {
            if (open_files[i].last_write_tick < oldest)
            {
                oldest = open_files[i].last_write_tick;
                lru_index = i;
            }
        }

        // found the oldest file
        file_log_t *oldestfile = &open_files[lru_index];
        ESP_LOGI(TAG, "CLOSED File: %s", oldestfile->fname);
        close_file(oldestfile);
        

        // now we have a spot for the new file
        // open file, fill out data_t and return
        file = &open_files[lru_index];
        file->fp = fopen(path, "a");
        if (!file->fp)
        {
            ESP_LOGE(TAG, "Failed to open file: %s", path);
            return NULL;
        }
    }
    snprintf(file->fname, sizeof(file->fname), "%s", path);
    file->index = 0;
    file->last_flush_tick = now;
    file->last_write_tick = now;
    num_open_files += 1;
    ESP_LOGI(TAG, "Opened file: %s", file->fname);
    return file;
}

//
// Add bytes to the write buffer of a file
// if buffer is going to fill, it will write to the file
//
static void file_buffer_write(file_log_t *file, const char *data, size_t len)
{
    if (!file || !file->fp) return;
    if (len > LOG_BUFFER_SIZE) len = LOG_BUFFER_SIZE;
    if (file->index + len > LOG_BUFFER_SIZE)
    {
        ESP_LOGI(TAG, "Dumping buffer len %d", file->index);
        fwrite(file->buffer, 1, file->index, file->fp);
        fflush(file->fp);
        file->index = 0;
        file->last_flush_tick = xTaskGetTickCount();
        file->last_write_tick = xTaskGetTickCount();
    }
    memcpy(&file->buffer[file->index], data, len);
    file->index += len;
    ESP_LOGI(TAG, "Added to buffer total len: %d", file->index);
    file->last_write_tick = xTaskGetTickCount();
}

//
// Call this and it will write the buffer to the file if its
//     been long enough
//
// Closes files if they've been open without a write for too long
//
static void flush_files_timer()
{
    TickType_t now = xTaskGetTickCount();
    for (int i = 0; i < MAX_OPEN_FILES; i++)
    {
        file_log_t *f = &open_files[i];

        if (f->fname[0] != '\0')
        {
            ESP_LOGI(TAG, "Checking %s for time", f->fname);
            if (f->index == 0) // buffer empty
            {
                // check the last time it was written, if too long ago close the file
                if ((now - f->last_write_tick) >= pdMS_TO_TICKS(MAX_FILE_HOLD_TIME_MS))
                {
                    // been too long with an empty buffer and no writing
                    // close file
                    close_file(f);
                }
            }

            if ((now - f->last_flush_tick) >= pdMS_TO_TICKS(LOG_FLUSH_INTERVAL_MS))
            {
                if (f->index > 0)
                {
                    fwrite(f->buffer, 1, f->index, f->fp);
                    fflush(f->fp);
                    f->index = 0;
                    f->last_flush_tick = now;
                }
            }
        }
    }
}

//
// Writes data to a file (buffer)
//
static esp_err_t write_file(const char *path, char *data)
{
    // ESP_LOGI(TAG, "Opening file %s", path);
    // FILE *f = fopen(path, "a");
    // if (f == NULL) {
    //     ESP_LOGE(TAG, "Failed to open file for writing");
    //     return ESP_FAIL;
    // }
    // fprintf(f, "%s", data);
    // fclose(f);
    // ESP_LOGI(TAG, "File written");
    
    // find the right struct from the pathname
    // this means call find or open
    // then with that struct add data to the buffer
    file_log_t *file = get_or_open_file(path);
    if (!file) {
        return ESP_FAIL;
    }

    file_buffer_write(file, data, strlen(data));
        return ESP_OK;
}

//
// THis is the esp idf example
// needs a rewrite to work in this context
// right now files in the open files array are open in append mode
//
// static esp_err_t read_file(const char *path)
// {
//     ESP_LOGI(TAG, "Reading file %s", path);
//     FILE *f = fopen(path, "r");
//     if (f == NULL) {
//         ESP_LOGE(TAG, "Failed to open file for reading");
//         return ESP_FAIL;
//     }
//     char line[512];
//     fgets(line, sizeof(line), f);
//     fclose(f);

//     // strip newline
//     char *pos = strchr(line, '\n');
//     if (pos) {
//         *pos = '\0';
//     }
//     ESP_LOGI(TAG, "Read from file: '%s'", line);

//     return ESP_OK;
// }

//
// Handles save requests from the queue
//
static int handle_save(save_req_t *save_req)
{
    esp_err_t good;
    ESP_LOGI(TAG, "GOT FNAME: %s", save_req->fname);
    char path[64];
    snprintf(path, sizeof(path), "%s/%s", MOUNT_POINT, save_req->fname);
    good = write_file(path, save_req->data);
    if (good != ESP_OK)
    {
        return 1;
    }
    return 0;
}

//
//
//
static void sd_task(void *arg)
{
    esp_err_t ret;
    save_req_t save_req;

    init_file_array();

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
    ret = write_file(file_hello, data);
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

        // check and empty file buffers
        flush_files_timer();
    }
}

//
//
//
void init_sd_task()
{

    uint64_t output_pin_mask = ((1ULL << SPI_CLK) | (1ULL << SPI_MOSI) | (1ULL << SPI_MISO) | (1ULL << SPI_CS0));
    gpio_config_t output_conf = {
        .pin_bit_mask = output_pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_conf);

    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << TOGGLE_SW),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&input_conf);



    save_queue = xQueueCreate(100, sizeof(save_req_t));


    xTaskCreatePinnedToCore(
        sd_task,
        "sd_task",
        8*4096,
        NULL,
        8,
        NULL,
        1
    );
}