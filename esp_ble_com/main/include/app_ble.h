/*
 * @Author: HoGC
 * @Date: 2024-02-25 20:21:05
 * @LastEditTime: 2024-03-14 23:04:52
 */
#ifndef __APP_BLE_H__
#define __APP_BLE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

enum
{
    APP_BLE_RECV_TYPE_DATA,
    APP_BLE_RECV_TYPE_COMMAND,
};

typedef void (* app_ble_recv_cb_t)(uint8_t type, uint8_t *data, uint16_t len);

esp_err_t app_ble_send(uint8_t *data, uint16_t len);

esp_err_t app_ble_init(app_ble_recv_cb_t recv_cb);

#ifdef __cplusplus
}
#endif

#endif