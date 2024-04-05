/*
 * @Author: HoGC
 * @Date: 2024-02-25 22:29:40
 * @LastEditTime: 2024-03-14 23:05:06
 */
#ifndef __APP_SERIAL_H__
#define __APP_SERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

typedef void (* app_serial_recv_cb_t)(uint8_t *data, uint16_t len);

void app_serial_set_dtr_rts(const bool dtr, const bool rts);

esp_err_t app_serial_set_baudrate(const uint32_t baud);

void app_serial_enable(const bool enable);

esp_err_t app_serial_send(uint8_t *data, uint16_t len);

esp_err_t app_serial_init(app_serial_recv_cb_t recv_cb);

#ifdef __cplusplus
}
#endif

#endif