/*
 * ble_ecg.c — NimBLE NUS (Nordic UART Service) peripheral for the ECG watch.
 *
 * Service  UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * TX char  UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (notify, watch → app)
 * RX char  UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (write,  app  → watch)
 *
 * NimBLE UUIDs are stored little-endian.
 */

#include "ble_ecg.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE";

// =========================================================================
// UUIDs (little-endian byte order for NimBLE)
// =========================================================================

/* 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
static ble_uuid128_t NUS_SVC_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* 6E400003 */
static ble_uuid128_t NUS_TX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

/* 6E400002 */
static ble_uuid128_t NUS_RX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

// =========================================================================
// State
// =========================================================================

static uint16_t s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_val_handle  = 0;
static bool     s_notify_enabled = false;

#define CMD_BUF_LEN 64
static char s_cmd_buf[CMD_BUF_LEN];
static bool s_cmd_ready = false;

// =========================================================================
// GATT access callbacks
// =========================================================================

static int nus_rx_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > 0 && len < CMD_BUF_LEN) {
            os_mbuf_copydata(ctxt->om, 0, len, s_cmd_buf);
            s_cmd_buf[len] = '\0';
            s_cmd_ready    = true;
            ESP_LOGI(TAG, "RX: %s", s_cmd_buf);
        }
    }
    return 0;
}

static int nus_tx_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)ctxt; (void)arg;
    return 0;   /* notify-only; reads return nothing */
}

// =========================================================================
// GATT service table
// NimBLE writes into both the service-def and characteristics arrays during
// registration (e.g. to store val_handle), so NEITHER can be const/flash.
// Declare them as separate static (RAM) variables.
// =========================================================================

static struct ble_gatt_chr_def s_nus_chars[] = {
    {   /* TX: watch → app via notify */
        .uuid       = &NUS_TX_UUID.u,
        .access_cb  = nus_tx_access,
        .val_handle = &s_tx_val_handle,
        .flags      = BLE_GATT_CHR_F_NOTIFY,
    },
    {   /* RX: app → watch via write */
        .uuid      = &NUS_RX_UUID.u,
        .access_cb = nus_rx_access,
        .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    { 0 }   /* terminator */
};

static struct ble_gatt_svc_def s_svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &NUS_SVC_UUID.u,
        .characteristics = s_nus_chars,
    },
    { 0 }   /* terminator */
};

// =========================================================================
// GAP event handler
// =========================================================================

static uint8_t s_own_addr_type = BLE_OWN_ADDR_RANDOM;

static void start_advertising(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle    = event->connect.conn_handle;
                s_notify_enabled = false;
                ESP_LOGI(TAG, "Connected handle=%d", s_conn_handle);
            } else {
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected reason=%d", event->disconnect.reason);
            s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
            s_notify_enabled = false;
            start_advertising();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_tx_val_handle) {
                s_notify_enabled = (event->subscribe.cur_notify != 0);
                ESP_LOGI(TAG, "Notify %s", s_notify_enabled ? "enabled" : "disabled");
            }
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU update: conn=%d mtu=%d",
                     event->mtu.conn_handle, event->mtu.value);
            break;

        default: break;
    }
    return 0;
}

// =========================================================================
// Advertising
// =========================================================================

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = { 0 };
    fields.flags             = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name              = (uint8_t *)"ECG-Watch";
    fields.name_len          = 9;
    fields.name_is_complete  = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields error %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = { 0 };
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                            &adv_params, gap_event, NULL);
    if (rc != 0) ESP_LOGE(TAG, "adv_start error %d", rc);
    else          ESP_LOGI(TAG, "Advertising as 'ECG-Watch'");
}

// =========================================================================
// NimBLE host task
// =========================================================================

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) ESP_LOGE(TAG, "ble_hs_id_infer_auto: %d", rc);
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset reason=%d", reason);
}

// =========================================================================
// Public API
// =========================================================================

void ble_ecg_init(void)
{
    nimble_port_init();

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_count_cfg: %d", rc); return; }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_add_svcs: %d", rc); return; }

    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "BLE ECG peripheral initialised");
}

bool ble_ecg_connected(void)
{
    return s_notify_enabled;
}

void ble_ecg_notify(uint8_t bpm, uint8_t arrhythmia, uint8_t battery)
{
    if (!s_notify_enabled) return;

    char buf[32];
    int  len = snprintf(buf, sizeof(buf), "H:%d,A:%d,B:%d\n",
                        (int)bpm, (int)arrhythmia, (int)battery);
    if (len <= 0) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, (uint16_t)len);
    if (!om) return;

    int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
    if (rc != 0) ESP_LOGD(TAG, "notify error %d", rc);
}

bool ble_ecg_get_command(char *buf, size_t len)
{
    if (!s_cmd_ready) return false;
    s_cmd_ready = false;
    strncpy(buf, s_cmd_buf, len - 1);
    buf[len - 1] = '\0';
    return true;
}

void ble_ecg_send_str(const char *msg, uint16_t len)
{
    if (!s_notify_enabled || len == 0) return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, len);
    if (!om) return;
    int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
    if (rc != 0) ESP_LOGD(TAG, "send_str notify error %d", rc);
}
