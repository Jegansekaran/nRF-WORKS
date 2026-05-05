/*
 * main.c — VW101 Vibrating Wire + BLE NUS (nRF52832)
 *
 * Changes from original:
 *   - Polling UART replaced with IRQ-driven RX + semaphore
 *   - UARTE suspended between reads via PM (releases HFCLK → low current)
 *   - bias-pull-up on RX pin (in overlay) prevents float-induced ISR storms
 *   - uart_flush_rx() removed (incompatible with IRQ path)
 *   - uart_recv_bytes() removed (replaced by k_sem_take timeout)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>          /* PM_DEVICE_ACTION_SUSPEND/RESUME */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <bluetooth/services/nus.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/gpio.h>

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

/* ================================================================
 * THREAD CONFIG
 * ================================================================ */

#define STACKSIZE        2048
#define PRIORITY_CALIB   6
#define PRIORITY_SEND    7

/* ================================================================
 * NVS IDs
 * ================================================================ */

#define EEPROM_TARE_ADDR   1
#define EEPROM_SCALE_ADDR  2
#define EEPROM_FLAG_ADDR   3
#define EEPROM_FLAG_UNIT   4
#define EEPROM_FLAG_INT    5
#define EEPROM_FLAG_DEC    6

#define CAL_DONE           0x55

/* ================================================================
 * VW101 MODBUS RTU DRIVER
 * ================================================================ */

#define VW101_UART_NODE   DT_ALIAS(vw101_uart)
#define VCC1              DT_NODELABEL(vcc1)
#define VCC2              DT_NODELABEL(vcc2)

#define MB_SLAVE_ADDR     0x01
#define MB_FC_READ        0x03

#define VW101_REG_FREQ    0x0000
#define VW101_REG_TEMP    0x0001

#define VW101_NOT_READ    0x0000
#define VW101_DISCONN     0xFFFF
#define VW101_BROKEN_WIRE 0xFFF3

#define VW101_TX_LEN      8
#define VW101_RX_LEN      9

#define VW101_RX_TIMEOUT_MS  4000

/* Pre-built request: 01 03 00 00 00 02 C4 0B */
static const uint8_t vw101_req[VW101_TX_LEN] = {
    MB_SLAVE_ADDR, MB_FC_READ,
    0x00, 0x00,
    0x00, 0x02,
    0xC4, 0x0B
};

static const struct device *vw101_uart_dev = NULL;

static const struct gpio_dt_spec vcc1_pin = GPIO_DT_SPEC_GET(VCC1, gpios);
static const struct gpio_dt_spec vcc2_pin = GPIO_DT_SPEC_GET(VCC2, gpios);

/* ================================================================
 * IRQ-DRIVEN RX STATE
 * ================================================================ */

static uint8_t          rx_buf[VW101_RX_LEN];
static volatile size_t  rx_count  = 0;
static volatile bool    rx_active = false;

K_SEM_DEFINE(rx_done_sem, 0, 1);

/**
 * vw101_uart_isr() — UART IRQ callback.
 *
 * Accumulates bytes into rx_buf. Signals rx_done_sem once
 * VW101_RX_LEN bytes have arrived. Discards bytes while
 * rx_active == false (inter-frame garbage / suspend period).
 */
static void vw101_uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        uint8_t byte;

        if (uart_fifo_read(dev, &byte, 1) <= 0) {
            break;
        }

        if (!rx_active) {
            continue;   /* discard — not expecting a frame */
        }

        if (rx_count < VW101_RX_LEN) {
            rx_buf[rx_count++] = byte;
        }

        if (rx_count == VW101_RX_LEN) {
            rx_active = false;
            k_sem_give(&rx_done_sem);
        }
    }
}

/* ================================================================
 * CRC-16/MODBUS  (poly 0xA001, init 0xFFFF)
 * ================================================================ */

static uint16_t crc16_modbus(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

/* ================================================================
 * TX helper — polling is fine for 8 bytes at 9600 baud (~8 ms)
 * ================================================================ */

static void uart_send_bytes(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(vw101_uart_dev, buf[i]);
    }
}

/* ================================================================
 * vw101_read_raw()
 *
 * Full sequence:
 *   1. Resume UARTE  → HFCLK starts, peripheral active
 *   2. Arm RX state
 *   3. Send MODBUS request
 *   4. Block on semaphore (ISR signals when 9 bytes arrive)
 *   5. Disable RX IRQ + Suspend UARTE → HFCLK released
 *   6. Validate frame + CRC
 *
 * Current profile:
 *   Idle  : ~20-50 µA  (UARTE suspended, BLE adv sleep)
 *   Active: ~1.2 mA    (UARTE + HFCLK on during 4 s capture)
 * ================================================================ */

static int vw101_read_raw(uint16_t *freq_raw, uint16_t *temp_raw)
{
    /* 1. Wake UARTE — re-enables peripheral clock */
    pm_device_action_run(vw101_uart_dev, PM_DEVICE_ACTION_RESUME);

    /* 2. Arm receiver BEFORE sending — no bytes can arrive before
     *    the request is sent (VW101 is request-response only) */
    rx_count  = 0;
    rx_active = true;
    k_sem_reset(&rx_done_sem);
    uart_irq_rx_enable(vw101_uart_dev);

    /* 3. Send MODBUS RTU request */
    uart_send_bytes(vw101_req, VW101_TX_LEN);

    /* 4. Block until 9 bytes collected or timeout */
    int ret = k_sem_take(&rx_done_sem, K_MSEC(VW101_RX_TIMEOUT_MS));

    /* 5. Suspend UARTE immediately — releases HFCLK */
    uart_irq_rx_disable(vw101_uart_dev);
    rx_active = false;   /* safety: disarm in case of timeout */
    pm_device_action_run(vw101_uart_dev, PM_DEVICE_ACTION_SUSPEND);

    if (ret != 0) {
        return -ETIMEDOUT;
    }

    /* 6. Validate frame header */
    if (rx_buf[0] != MB_SLAVE_ADDR || rx_buf[1] != MB_FC_READ) {
        return -EIO;
    }

    /* Validate CRC (covers bytes 0..6) */
    uint16_t crc_calc = crc16_modbus(rx_buf, VW101_RX_LEN - 2);
    uint16_t crc_recv = (uint16_t)rx_buf[7] | ((uint16_t)rx_buf[8] << 8);
    if (crc_calc != crc_recv) {
        return -EBADMSG;
    }

    *freq_raw = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];
    if (temp_raw) {
        *temp_raw = ((uint16_t)rx_buf[5] << 8) | rx_buf[6];
    }

    return 0;
}

/* ================================================================
 * GLOBALS
 * ================================================================ */

float    K            = 0.00035f;
float    tare         = 0.0f;
uint16_t integer_part = 0;
uint16_t decimal_part = 0;

static struct nvs_fs fs;

/* ================================================================
 * SEMAPHORES
 * ================================================================ */

K_SEM_DEFINE(calib_done_sem,    0, 1);
K_SEM_DEFINE(ble_connected_sem, 0, 1);
K_SEM_DEFINE(init_calib_sem,    0, 1);
K_SEM_DEFINE(load_written,      0, 1);
K_SEM_DEFINE(calib_stop,        0, 1);

/* ================================================================
 * ATOMICS
 * ================================================================ */

static atomic_t connected       = ATOMIC_INIT(0);
static atomic_t freq_received   = ATOMIC_INIT(0);
static atomic_t strain_received = ATOMIC_INIT(0);

/* ================================================================
 * BLE GLOBALS
 * ================================================================ */

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

struct bt_conn *my_conn = NULL;

static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),
    4800, 4850, NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* ================================================================
 * NVS INIT
 * ================================================================ */

static void nvs_init(void)
{
    struct flash_pages_info info;

    fs.flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    fs.offset       = FIXED_PARTITION_OFFSET(storage_partition);

    flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);

    fs.sector_size  = info.size;
    fs.sector_count = FIXED_PARTITION_SIZE(storage_partition) / fs.sector_size;

    nvs_mount(&fs);
}

/* ================================================================
 * SENSOR FUNCTIONS
 * ================================================================ */

static uint16_t hex_value(void)
{
    uint16_t freq_raw = 0;

    int ret = vw101_read_raw(&freq_raw, NULL);
    if (ret != 0) {
        return 0;
    }

    if (freq_raw == VW101_NOT_READ  ||
        freq_raw == VW101_DISCONN   ||
        freq_raw == VW101_BROKEN_WIRE) {
        return 0;
    }

    return freq_raw;
}

static bool hex_value_with_temp(float *freq_hz, float *temp_c)
{
    uint16_t freq_raw = 0, temp_raw = 0;

    int ret = vw101_read_raw(&freq_raw, &temp_raw);
    if (ret != 0) {
        return false;
    }

    if (freq_raw == VW101_NOT_READ  ||
        freq_raw == VW101_DISCONN   ||
        freq_raw == VW101_BROKEN_WIRE) {
        return false;
    }

    *freq_hz = (float)freq_raw / 10.0f;
    *temp_c  = (temp_raw == VW101_DISCONN)
               ? 0.0f
               : (((float)(int16_t)temp_raw - 500.0f) / 10.0f);

    return true;
}

static void offset_value(void)
{
    uint16_t raw = hex_value();
    tare = (float)raw / 10.0f;
    nvs_write(&fs, EEPROM_TARE_ADDR, &tare, sizeof(tare));
}

static float final_value(void)
{
    uint16_t raw_counts = hex_value();

    if (raw_counts == 0) {
        bt_nus_send(NULL, "LOW SIGNAL RETRY", 16);
        return 0.0f;
    }

    float raw_hz = (float)raw_counts / 10.0f;

    nvs_read(&fs, EEPROM_TARE_ADDR, &tare, sizeof(tare));

    if (atomic_get(&freq_received)) {
        return raw_hz;
    }

    if (atomic_get(&strain_received)) {
        float delta = raw_hz - tare;
        return K * delta * (raw_hz + tare);
    }

    return 0.0f;
}

static float k_value(void)
{
    nvs_read(&fs, EEPROM_FLAG_INT, &integer_part, sizeof(integer_part));
    nvs_read(&fs, EEPROM_FLAG_DEC, &decimal_part, sizeof(decimal_part));

    if (integer_part == 0 && decimal_part == 0) {
        return 0.00035f;
    }

    return (float)integer_part * powf(10.0f, -(float)decimal_part);
}

/* ================================================================
 * BLE COMMAND RECEIVER
 * ================================================================ */

static void nus_received(struct bt_conn *conn,
                         const uint8_t *const data,
                         uint16_t len)
{
    if (len < 4) {
        return;
    }

    int val = (int)((uint32_t)data[0]         |
                    ((uint32_t)data[1] <<  8)  |
                    ((uint32_t)data[2] << 16)  |
                    ((uint32_t)data[3] << 24));

    uint8_t cmd = (uint32_t)val >> 28;

    switch (cmd) {

    case 0x0:
        integer_part = (uint16_t)val;
        nvs_write(&fs, EEPROM_FLAG_INT, &integer_part, sizeof(integer_part));
        break;

    case 0x5:
        decimal_part = (uint16_t)(val & 0x0FFFFFFF);
        nvs_write(&fs, EEPROM_FLAG_DEC, &decimal_part, sizeof(decimal_part));
        break;

    case 0x1:
        atomic_set(&strain_received, 0);
        atomic_set(&freq_received,   1);
        k_sem_give(&load_written);
        k_sem_give(&calib_done_sem);
        break;

    case 0x2:
        atomic_set(&freq_received,   0);
        atomic_set(&strain_received, 1);
        k_sem_give(&load_written);
        k_sem_give(&calib_done_sem);
        break;

    case 0x3:
        k_sem_give(&init_calib_sem);
        k_sem_give(&calib_stop);
        break;

    default:
        break;
    }
}

/* ================================================================
 * CALIBRATION THREAD
 * ================================================================ */

void calibration_thread(void)
{
    k_sem_take(&ble_connected_sem, K_FOREVER);

    char txt[64];

    while (1) {
        k_sem_take(&calib_stop,     K_FOREVER);
        k_sem_take(&init_calib_sem, K_FOREVER);

        nvs_read(&fs, EEPROM_TARE_ADDR, &tare, sizeof(tare));

        gpio_pin_set_dt(&vcc1_pin, 1);
        gpio_pin_set_dt(&vcc2_pin, 1);
        k_msleep(2000);

        offset_value();
        K = k_value();

        snprintf(txt, sizeof(txt), "F0 = %ld Hz", (long)tare);
        bt_nus_send(NULL, txt, strlen(txt));
        k_msleep(100);

        int32_t k_int = (int32_t)(K * 1e6f);
        snprintf(txt, sizeof(txt), "K = %de-6", k_int);
        bt_nus_send(NULL, txt, strlen(txt));
        k_msleep(100);

        gpio_pin_set_dt(&vcc1_pin, 0);
        gpio_pin_set_dt(&vcc2_pin, 0);
    }
}

/* ================================================================
 * SEND THREAD
 * ================================================================ */

void send_thread(void)
{
    char  tx_buf[64];
    float freq_hz = 0.0f, temp_c = 0.0f;

    k_sem_take(&calib_done_sem, K_FOREVER);

    while (1) {
        k_sem_take(&load_written, K_FOREVER);

        gpio_pin_set_dt(&vcc1_pin, 1);
        gpio_pin_set_dt(&vcc2_pin, 1);
        k_msleep(2000);

        bool ok = hex_value_with_temp(&freq_hz, &temp_c);

        if (!ok) {
            bt_nus_send(NULL, "LOW SIGNAL RETRY", 16);
            k_msleep(500);
            gpio_pin_set_dt(&vcc1_pin, 0);
            gpio_pin_set_dt(&vcc2_pin, 0);
            k_sem_give(&load_written);   /* keep retrying */
            continue;
        }

        nvs_read(&fs, EEPROM_TARE_ADDR, &tare, sizeof(tare));

        if (atomic_get(&freq_received)) {

            int32_t f_int  = (int32_t)freq_hz;
            int32_t f_frac = (int32_t)((freq_hz - (float)f_int) * 100.0f);

            snprintf(tx_buf, sizeof(tx_buf),
                     "F:%ld.%02ld Hz\r\n",
                     (long)f_int, (long)f_frac);

            atomic_set(&freq_received, 0);

        } else if (atomic_get(&strain_received)) {

            float delta  = freq_hz - tare;
            float strain = K * delta * (freq_hz + tare);

            int32_t s_int  = (int32_t)strain;
            int32_t s_frac = (int32_t)fabsf((strain - (float)s_int) * 100.0f);

            snprintf(tx_buf, sizeof(tx_buf),
                     "ST:%ld.%02ld\r\n",
                     (long)s_int, (long)s_frac);

            atomic_set(&strain_received, 0);

        } else {
            gpio_pin_set_dt(&vcc1_pin, 0);
            gpio_pin_set_dt(&vcc2_pin, 0);
            continue;
        }

        bt_nus_send(NULL, tx_buf, strlen(tx_buf));

        gpio_pin_set_dt(&vcc1_pin, 0);
        gpio_pin_set_dt(&vcc2_pin, 0);
    }
}

/* ================================================================
 * BLE CONNECTION CALLBACKS
 * ================================================================ */

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    my_conn = bt_conn_ref(conn);
    atomic_set(&connected, 1);
    k_sem_give(&ble_connected_sem);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    if (my_conn) {
        bt_conn_unref(my_conn);
        my_conn = NULL;
    }
    atomic_set(&connected, 0);
}

struct bt_conn_cb connection_callbacks = {
    .connected    = on_connected,
    .disconnected = disconnected_cb,
};

/* ================================================================
 * MAIN
 * ================================================================ */

int main(void)
{
    int err;

    /* ---- BLE ---- */
    err = bt_enable(NULL);
    if (err) {
        return -1;
    }

    bt_conn_cb_register(&connection_callbacks);

    static struct bt_nus_cb nus_cb = {
        .received = nus_received,
    };
    bt_nus_init(&nus_cb);
    bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

    /* ---- NVS ---- */
    nvs_init();

    /* ---- GPIO ---- */
    gpio_pin_configure_dt(&vcc1_pin, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&vcc2_pin, GPIO_OUTPUT_LOW);
    gpio_pin_set_dt(&vcc1_pin, 0);
    gpio_pin_set_dt(&vcc2_pin, 0);

    /* ---- VW101 UART ---- */
    vw101_uart_dev = DEVICE_DT_GET(VW101_UART_NODE);
    if (!device_is_ready(vw101_uart_dev)) {
        return -1;
    }

    /* Register ISR */
    uart_irq_callback_user_data_set(vw101_uart_dev, vw101_uart_isr, NULL);

    /*
     * Start UARTE suspended so it does not hold HFCLK.
     * vw101_read_raw() will resume it only during an active read.
     * uart_irq_rx_enable() is called inside vw101_read_raw() after resume.
     */
    pm_device_action_run(vw101_uart_dev, PM_DEVICE_ACTION_SUSPEND);

    /* ---- Restore calibration from NVS ---- */
    K = k_value();
    nvs_read(&fs, EEPROM_TARE_ADDR, &tare, sizeof(tare));

    return 0;
}

/* ================================================================
 * THREAD DEFINITIONS
 * ================================================================ */

K_THREAD_DEFINE(calib_id,
                STACKSIZE,
                calibration_thread,
                NULL, NULL, NULL,
                PRIORITY_CALIB,
                0, 0);

K_THREAD_DEFINE(send_id,
                STACKSIZE,
                send_thread,
                NULL, NULL, NULL,
                PRIORITY_SEND,
                0, 0);