/*
 * @Author: HoGC
 * @Date: 2024-02-25 18:30:21
 * @LastEditTime: 2024-03-14 23:05:54
 */
#include "app_ble.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"

#include "driver/uart.h"

#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56
#define SAMPLE_DEVICE_NAME "ESP_BLE_UART"
#define SPP_SVC_INST_ID 0

#define SPP_DATA_MAX_LEN (512)
#define SPP_CMD_MAX_LEN (20)

#define SEND_RINGBUFFER_SIZE (2 * 1024)

static const char *TAG = "app_ble";

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_NOTIFY 0xFFF1
#define ESP_GATT_UUID_SPP_DATA_RECEIVE 0xFFF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE 0xFFF3

static RingbufHandle_t g_send_ringbuf = NULL;

static uint16_t g_spp_mtu_size = 23;
static uint16_t g_spp_conn_id = 0xffff;
static esp_gatt_if_t g_spp_gatts_if = 0xff;

static app_ble_recv_cb_t g_recv_cb = NULL;

// Attributes State Machine
enum
{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,
    SPP_IDX_SPP_COMMAND_CFG,

    SPP_IDX_NB,
};

static const uint8_t spp_adv_data[] = {
    /* Flags */
    2,
    0x01,
    0x06,
    11,
    0xFF,
    0x57,
    0xAB,
    0x07,
    0x01,
    'C',
    'H',
    '9',
    '1',
    '4',
    '3',
    // 12,0xFF,0x57,0xAB,0x07,0x01,'E','S','P','3','2','C','3',
    9,
    0xFF,
    0x0C,
    0xE7,
    0x21,
    0x88,
    0x00,
    0x00,
    0x00,
    0x00,
};

static uint8_t spp_ascan_rsp_data[] = {
    5,
    0x12,
    0x08,
    0x00,
    0x0A,
    0x00,
    2,
    0x0A,
    0x00,
    13,
    0x09,
    'E',
    'S',
    'P',
    '_',
    'B',
    'L',
    'E',
    '_',
    'U',
    'A',
    'R',
    'T',
};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_write_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_write_nr_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/// SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t spp_data_receive_val[20] = {0x00};

/// SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t spp_data_notify_val[20] = {0x00};
static const uint8_t spp_data_notify_ccc[2] = {0x00, 0x00};

/// SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t spp_command_val[20] = {0x00};
static const uint8_t spp_command_ccc[2] = {0x00, 0x00};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
    {
        // SPP -  Service Declaration
        [SPP_IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

        // SPP -  data notify characteristic Declaration
        [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // SPP -  data notify characteristic Value
        [SPP_IDX_SPP_DATA_NTY_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ, SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

        // SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_DATA_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

        // SPP -  data receive characteristic Declaration
        [SPP_IDX_SPP_DATA_RECV_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_write_nr}},

        // SPP -  data receive characteristic Value
        [SPP_IDX_SPP_DATA_RECV_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_DATA_MAX_LEN, sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

        // SPP -  command characteristic Declaration
        [SPP_IDX_SPP_COMMAND_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_write_nr_notify}},

        // SPP -  command characteristic Value
        [SPP_IDX_SPP_COMMAND_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_CMD_MAX_LEN, sizeof(spp_command_val), (uint8_t *)spp_command_val}},

        // SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_COMMAND_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_command_ccc), (uint8_t *)spp_command_ccc}},
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB; i++)
    {
        if (handle == spp_handle_table[i])
        {
            return i;
        }
    }

    return error;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res = 0xff;

    // ESP_LOGI(TAG, "event = %x\n",event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));
        esp_ble_gap_config_scan_rsp_data_raw((uint8_t *)spp_ascan_rsp_data, sizeof(spp_ascan_rsp_data));
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
        break;
    case ESP_GATTS_READ_EVT:
        res = find_char_and_desr_index(p_data->read.handle);
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        res = find_char_and_desr_index(p_data->write.handle);
        if (p_data->write.is_prep == false)
        {
            if (res == SPP_IDX_SPP_COMMAND_VAL)
            {
                // ESP_LOG_BUFFER_HEXDUMP(TAG, (char *)p_data->write.value,p_data->write.len, ESP_LOG_INFO);
                if (g_recv_cb)
                {
                    g_recv_cb(APP_BLE_RECV_TYPE_COMMAND, p_data->write.value, p_data->write.len);
                }
            }
            else if (res == SPP_IDX_SPP_DATA_RECV_VAL)
            {
                if (g_recv_cb)
                {
                    g_recv_cb(APP_BLE_RECV_TYPE_DATA, p_data->write.value, p_data->write.len);
                }
            }
            else
            {
                // TODO:
            }
        }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        g_spp_mtu_size = p_data->mtu.mtu;
        ESP_LOGI(TAG, "g_spp_mtu_size = %d", g_spp_mtu_size);
        break;
    case ESP_GATTS_CONNECT_EVT:
        g_spp_conn_id = p_data->connect.conn_id;
        g_spp_gatts_if = gatts_if;
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        g_spp_conn_id = 0xffff;
        g_spp_gatts_if = 0xff;
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
        }
        else if (param->add_attr_tab.num_handle != SPP_IDX_NB)
        {
        }
        else
        {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
        }
        break;
    }
    default:
        break;
    }
}

static void _gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if)
            {
                if (spp_profile_tab[idx].gatts_cb)
                {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void _gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void _ble_sender_task(void *pvParameters)
{
    while (1)
    {
        size_t recv_len;
        uint8_t *buf = (uint8_t *)xRingbufferReceiveUpTo(g_send_ringbuf, &recv_len, pdMS_TO_TICKS(100),
                                                         g_spp_mtu_size - 3);

        if (buf)
        {
            uint8_t int_buf[514];
            memcpy(int_buf, buf, recv_len);
            vRingbufferReturnItem(g_send_ringbuf, (void *)buf);
            if (g_spp_conn_id != 0xffff)
            {
                esp_ble_gatts_send_indicate(g_spp_gatts_if, g_spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], recv_len, int_buf, false);
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
    }
    vTaskDelete(NULL);
}

esp_err_t app_ble_send(uint8_t *data, uint16_t len)
{

    if (g_spp_conn_id == 0xffff)
    {
        return ESP_FAIL;
    }

    if (xRingbufferSend(g_send_ringbuf, data, len, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGV(TAG, "Cannot write to ringbuffer!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t app_ble_init(app_ble_recv_cb_t recv_cb)
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "%s init bluetooth\n", __func__);
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    esp_ble_gatts_register_callback(_gatts_event_handler);
    esp_ble_gap_register_callback(_gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    g_send_ringbuf = xRingbufferCreate(SEND_RINGBUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (g_send_ringbuf)
    {
        xTaskCreate(_ble_sender_task, "ble_sender_task", 3 * 1024, NULL, 5, NULL);
    }
    else
    {
        ESP_LOGE(TAG, "Cannot create ringbuffer");
        return ESP_FAIL;
    }

    g_recv_cb = recv_cb;

    return ESP_OK;
}