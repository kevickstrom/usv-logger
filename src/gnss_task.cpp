#include "gnss_task.h"
#include "uart_manager.h"
#include "config.h"
#include "esp_log.h"
#include "qqqlab_GPS_UBLOX.h"
#include <string.h>

static const char *TAG = "GNSS_TASK";
uart_transaction_t trans;  // Holds the transaction

// ---------------- GPS Interface ----------------
class GPS_Interface_IDF : public AP_GPS_UBLOX {
public:
    void I_setBaud(int) override {}

    int I_available() override {
        return rx_len;
    }

    int I_read(uint8_t *data, size_t len) override {
        if (rx_len == 0) return 0;

        // Only read 1 byte at a time (parser expects stream)
        *data = rx_buf[0];
        rx_len--;
        memmove(rx_buf, rx_buf + 1, rx_len);
        return 1;
    }

    int I_write(uint8_t *data, size_t len) override {
        if(len > sizeof(trans.tx_buf)) return 0;

        // Prepare UART transaction
        memset(&trans, 0, sizeof(trans));
        memcpy(trans.tx_buf, data, len);
        trans.tx_len = len;
        trans.device = PING;
        trans.timeout_ms = 500;
        trans.caller = xTaskGetCurrentTaskHandle();  // Notifies GNSS task

        uart_transaction_t *ptr = &trans;
        if (xQueueSend(get_uart_queue(), &ptr, portMAX_DELAY) != pdPASS) return 0;

        // Wait for transaction to complete (GNSS task will block here)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Feed parser **incrementally** from received bytes
        rx_len = 0;
        for (size_t i = 0; i < trans.rx_len; i++) {
            if (rx_len < sizeof(rx_buf)) {
                rx_buf[rx_len++] = trans.rx_buf[i];
            }
        }
        return len;
    }

    int I_availableForWrite() override {
        return 128;  // Arbitrary, allow writes anytime
    }

    uint32_t I_millis() override {
        return xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    void I_print(const char *str) override {
        ESP_LOGI(TAG, "%s", str);
    }

private:
    uint8_t rx_buf[512]{};  // Buffer to store GPS data
    size_t rx_len = 0;
};

// ---------------- GNSS Task ----------------
static GPS_Interface_IDF gps;

static void send_general_request(uart_transaction_t *trans, uint16_t requested_id) {
    memset(trans, 0, sizeof(*trans));

    trans->device = PING;
    trans->caller = xTaskGetCurrentTaskHandle();
    trans->timeout_ms = 500;

    trans->tx_buf[0] = requested_id & 0xFF;
    trans->tx_buf[1] = (requested_id >> 8) & 0xFF;
    trans->tx_len = 2;

    uart_transaction_t *ptr = trans;  // Pointer to static struct
    if (xQueueSend(get_uart_queue(), &ptr, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send UART transaction");
        return;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait until the transaction is complete
}

// ---------------- GNSS Task ----------------
void gnss_task(void *arg) {
    uint8_t gps_data[512];  // Buffer to store GPS data (assuming 512 bytes for now)

    ESP_LOGI(TAG, "GNSS task started");

    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        ESP_LOGI(TAG, "Sending GPS request...");
        // Build and send a request transaction
        uint8_t msg[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0x00,0x00,0x0A,0x0D};  // Example message
        gps.I_write(msg, sizeof(msg));

        // Now repeatedly call update() to parse bytes
        gps.update();  // Update GPS data from the serial stream

        // Print GPS state (e.g., position)
        ESP_LOGI(TAG, "tow:%d dt:%d sats:%d lat:%d lng:%d alt:%d hacc:%d vacc:%d fix:%d\n"
                        , (int)gps.state.time_week_ms
                        , (int)gps.timing.average_delta_us
                        , (int)gps.state.num_sats
                        , (int)gps.state.lat
                        , (int)gps.state.lng
                        , (int)gps.state.alt
                        , (int)gps.state.horizontal_accuracy
                        , (int)gps.state.vertical_accuracy   
                        , (int)gps.state.status
        );

        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for 1 second before next update
    }
}

// ---------------- GNSS Task Init ----------------
void init_gnss_task() {
    xTaskCreatePinnedToCore(
        gnss_task,
        "gnss",
        8192,
        NULL,
        8,
        NULL,
        0
    );
}