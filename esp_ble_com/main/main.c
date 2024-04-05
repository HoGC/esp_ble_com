/*
 * @Author: HoGC
 * @Date: 2024-02-25 18:23:16
 * @LastEditTime: 2024-04-05 17:23:34
 */
#include <stdio.h>
#include <inttypes.h>

#include "sdkconfig.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_ble.h"
#include "app_serial.h"

typedef struct
{
    uint32_t cmd;
    union
    {
        struct
        {
            uint32_t baudrate;
        } baudrate_cfg;
        struct
        {
            uint8_t unknown;
            uint8_t dtr;
            uint8_t rts;
        } dtr_rts_cfg;
    } data;
} _command;

void _ble_recv_cb(uint8_t type, uint8_t *data, uint16_t len)
{
    if (type == APP_BLE_RECV_TYPE_DATA)
    {
        app_serial_send(data, len);
    }
    else if (type == APP_BLE_RECV_TYPE_COMMAND)
    {
        _command *command = (_command *)data;
        if (command->cmd == 0x090006)
        {
            app_serial_set_baudrate(command->data.baudrate_cfg.baudrate);
        }
        else if (command->cmd == 0x050007)
        {
            app_serial_set_dtr_rts(command->data.dtr_rts_cfg.dtr, command->data.dtr_rts_cfg.rts);
        }
    }
}

void _serial_recv_cb(uint8_t *data, uint16_t len)
{
    app_ble_send(data, len);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    app_ble_init(_ble_recv_cb);
    app_serial_init(_serial_recv_cb);
}
