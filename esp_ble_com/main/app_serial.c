/*
 * @Author: HoGC
 * @Date: 2024-02-25 22:23:25
 * @LastEditTime: 2024-03-14 23:06:02
 */
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "app_serial.h"

#include "esp_idf_version.h"
#include "esp_loader.h"
#include "esp32_port.h"
#include "esp_log.h"

#define GPIO_BOOT CONFIG_COM_GPIO_BOOT
#define GPIO_RST CONFIG_COM_GPIO_RST
#define GPIO_RXD CONFIG_COM_GPIO_RXD
#define GPIO_TXD CONFIG_COM_GPIO_TXD

#define SLAVE_UART_BUF_SIZE (2 * 1024)
#define SLAVE_UART_DEFAULT_BAUD 115200
#define SLAVE_UART_NUM UART_NUM_1

static const char *TAG = "bridge_serial";

static QueueHandle_t g_uart_queue;
static atomic_bool g_serial_read_enabled = false;

static app_serial_recv_cb_t g_recv_cb = NULL;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t dtmp[SLAVE_UART_BUF_SIZE];

    while (1)
    {
        if (xQueueReceive(g_uart_queue, (void *)&event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
                if (g_serial_read_enabled)
                {
                    size_t buffered_len;
                    uart_get_buffered_data_len(SLAVE_UART_NUM, &buffered_len);
                    const int read = uart_read_bytes(SLAVE_UART_NUM, dtmp, MIN(buffered_len, SLAVE_UART_BUF_SIZE), portMAX_DELAY);
                    // ESP_LOGD(TAG, "UART -> CDC ringbuffer (%d bytes)", read);
                    // ESP_LOG_BUFFER_HEXDUMP("UART -> CDC", dtmp, read, ESP_LOG_DEBUG);
                    if (g_recv_cb)
                    {
                        g_recv_cb(dtmp, read);
                    }
                }
                break;
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "UART FIFO overflow");
                uart_flush_input(SLAVE_UART_NUM);
                xQueueReset(g_uart_queue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART ring buffer full");
                uart_flush_input(SLAVE_UART_NUM);
                xQueueReset(g_uart_queue);
                break;
            case UART_BREAK:
                ESP_LOGW(TAG, "UART RX break");
                break;
            case UART_PARITY_ERR:
                ESP_LOGW(TAG, "UART parity error");
                break;
            case UART_FRAME_ERR:
                ESP_LOGW(TAG, "UART frame error");
                break;
            default:
                ESP_LOGW(TAG, "UART event type: %d", event.type);
                break;
            }
            taskYIELD();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void app_serial_set_dtr_rts(const bool dtr, const bool rts)
{
    // defaults for ((dtr && rts) || (!dtr && !rts))
    bool rst = true;
    bool boot = true;

    rst = !rts;
    boot = !dtr;

    ESP_LOGI(TAG, "DTR = %d, RTS = %d -> BOOT = %d, RST = %d", dtr, rts, boot, rst);

    gpio_set_level(GPIO_BOOT, boot);
    gpio_set_level(GPIO_RST, rst);
}

esp_err_t app_serial_set_baudrate(const uint32_t baud)
{
    ESP_LOGI(TAG, "baudrate = %ld", baud);

    return uart_set_baudrate(SLAVE_UART_NUM, baud);
}

void app_serial_enable(const bool enable)
{
    g_serial_read_enabled = enable;
}

esp_err_t app_serial_send(uint8_t *data, uint16_t len)
{
    return uart_write_bytes(SLAVE_UART_NUM, data, len);
}

esp_err_t app_serial_init(app_serial_recv_cb_t recv_cb)
{
    const loader_esp32_config_t serial_conf = {
        .baud_rate = SLAVE_UART_DEFAULT_BAUD,
        .uart_port = SLAVE_UART_NUM,
        .uart_rx_pin = GPIO_RXD,
        .uart_tx_pin = GPIO_TXD,
        .rx_buffer_size = SLAVE_UART_BUF_SIZE * 2,
        .tx_buffer_size = 0,
        .uart_queue = &g_uart_queue,
        .queue_size = 20,
        .reset_trigger_pin = GPIO_RST,
        .gpio0_trigger_pin = GPIO_BOOT,
    };

    if (loader_port_esp32_init(&serial_conf) == ESP_LOADER_SUCCESS)
    {
        ESP_LOGI(TAG, "UART & GPIO have been initialized");

        gpio_set_level(GPIO_RST, 1);
        gpio_set_level(GPIO_BOOT, 1);

        g_serial_read_enabled = true;
    }
    else
    {
        ESP_LOGE(TAG, "loader_port_serial_init failed");
        return ESP_FAIL;
    }

    xTaskCreate(uart_event_task, "uart_event_task", 8 * 1024, NULL, 5, NULL);

    g_recv_cb = recv_cb;

    return ESP_OK;
}
