#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/logging/log.h>
#include <bluetooth/services/nus.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(hx711_ble, LOG_LEVEL_INF);

/* ============================================================
 * CONFIG
 * ============================================================ */
#define STACKSIZE        1536
#define PRIORITY_UART    5
#define PRIORITY_SEND    6
#define PRIORITY_LOG     7
#define NOTIFY_INTERVAL  170

#define HX711_GAIN       128.0f
#define HX711_FSR        8388607.0f   /* 2^23 - 1 */
#define AVDD_MV_DEFAULT  2150.0f      /* measured AVDD — update if remeasured */

/* ============================================================
 * KERNEL OBJECTS
 * ============================================================ */
K_SEM_DEFINE(stream_sem, 0, 1);
K_SEM_DEFINE(log_sem,    0, 1);
K_MUTEX_DEFINE(data_mutex);

/* ============================================================
 * LOG TIMER  (RTC-backed via Zephyr kernel timer on nRF52)
 *
 * On nRF52, k_timer is backed by hardware RTC1 through
 * Zephyr's tick subsystem — no separate RTC driver needed
 * unless persistence across SYSTEM_OFF deep sleep is required.
 * ============================================================ */
static void log_timer_expiry(struct k_timer *t)
{
    ARG_UNUSED(t);
    if (atomic_get(&(*(atomic_t *)k_timer_user_data_get(t)))) {
        k_sem_give(&log_sem);
    }
}

static void log_timer_stop_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
}

K_TIMER_DEFINE(log_interval_timer, log_timer_expiry, log_timer_stop_cb);

/* ============================================================
 * SHARED STATE
 * ============================================================ */
static atomic_t connected       = ATOMIC_INIT(0);
static atomic_t streaming       = ATOMIC_INIT(0);
static atomic_t channel         = ATOMIC_INIT(0);
static atomic_t channel_changed = ATOMIC_INIT(0);
static atomic_t sel_all         = ATOMIC_INIT(0);
static atomic_t log_write       = ATOMIC_INIT(0);
static atomic_t log_read        = ATOMIC_INIT(0);

/* Protected by data_mutex */
static float    avdd_mv  = AVDD_MV_DEFAULT;
static uint16_t timer    = 0;
static int64_t  tared[4] = {0};

static bool log_full[4] = {false};

/* ============================================================
 * WALL-CLOCK TIME  (Option B — UART-set, no external RTC chip)
 *
 * boot_epoch_ms = ms-since-midnight at the moment the user
 *                 typed the current time via UART.
 *
 * Current wall-clock ms =  boot_epoch_ms + k_uptime_get()
 *
 * Limitations:
 *   - Must be re-entered after every power cycle.
 *   - Drifts at ~±20 ppm (32.768 kHz XTAL accuracy).
 *   - Wraps at midnight (handled in get_time_str).
 * ============================================================ */
static int64_t boot_epoch_ms = -1;   /* -1 = time not yet set */

/*
 * get_time_str()
 *
 * Fills `out` with "HH:MM:SS" if time has been set, or
 * "T+<ms>" (raw uptime) if the user has not set the time yet.
 */
static void get_time_str(char *out, size_t len)
{

    int64_t now_ms  = boot_epoch_ms + k_uptime_get();
    int64_t now_sec = (now_ms / 1000) % 86400;   /* wrap at midnight */

    int h = (int)(now_sec / 3600);
    int m = (int)((now_sec % 3600) / 60);
    int s = (int)(now_sec % 60);

    snprintf(out, len, "%02d:%02d:%02d", h, m, s);
}

/* ============================================================
 * NVS LAYOUT
 *   ID  5          : session counter
 *   IDs 10-13      : persisted log_idx per channel
 *   IDs 20-999     : CH0 log entries  (980 max)
 *   IDs 1000-1999  : CH1 log entries
 *   IDs 2000-2999  : CH2 log entries
 *   IDs 3000-3999  : CH3 log entries
 * ============================================================ */
static const uint16_t log_base[4]    = {20,   1000, 2000, 3000}; // nvs_id  = log_base + log_idx this is the id where the log entry is stored 
static const uint16_t log_idx_nvs[4] = {10,   11,   12,   13  }; // this the id to store the current index
static const uint16_t log_max[4]     = {980,  1000, 1000, 1000};

#define NVS_ID_SESSION   5 //id for session counter

#define NVS_ID_TIME   6   //id for initial time

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* ============================================================
 * DATA STRUCTURES
 * ============================================================ */
struct vw_log_entry {
    uint32_t uptime_ms;
    float    mv_per_v;
};

static struct vw_log_entry log_data[4];

static struct nvs_fs fs;

struct bt_conn *my_conn = NULL;

/* Session counter — incremented on every boot, stored in NVS */
static uint16_t session = 0;

/* ============================================================
 * HX711 GPIO
 * ============================================================ */
struct hx711_dev {
    struct gpio_dt_spec dt;
    struct gpio_dt_spec sck;
};

static struct hx711_dev hx[4] = {
    {
        .dt  = GPIO_DT_SPEC_GET(DT_NODELABEL(dt_0),  gpios),
        .sck = GPIO_DT_SPEC_GET(DT_NODELABEL(sck_0), gpios),
    },
    {
        .dt  = GPIO_DT_SPEC_GET(DT_NODELABEL(dt_1),  gpios),
        .sck = GPIO_DT_SPEC_GET(DT_NODELABEL(sck_1), gpios),
    },
    {
        .dt  = GPIO_DT_SPEC_GET(DT_NODELABEL(dt_2),  gpios),
        .sck = GPIO_DT_SPEC_GET(DT_NODELABEL(sck_2), gpios),
    },
    {
        .dt  = GPIO_DT_SPEC_GET(DT_NODELABEL(dt_3),  gpios),
        .sck = GPIO_DT_SPEC_GET(DT_NODELABEL(sck_3), gpios),
    }
};

static const struct device *uart_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* ============================================================
 * BLE ADV
 * ============================================================ */
static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),
    4800, 4850, NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                  (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE,
            DEVICE_NAME,
            DEVICE_NAME_LEN),
};

/* ============================================================
 * UART HELPERS
 * ============================================================ */
static void uart_send_str(const char *str)
{
    while (*str) {
        uart_poll_out(uart_dev, *str++);
    }
}

/* Read a line from UART — UART thread only, never call from BLE context */
static bool uart_read_line(char *buf, uint8_t max_len)
{
    uint8_t idx = 0;

    while (1) {
        unsigned char c;
        while (uart_poll_in(uart_dev, &c) != 0) {
            k_sleep(K_MSEC(1));
        }

        if (c == '\r' || c == '\n') {
            if (idx > 0) {
                buf[idx] = '\0';
                return true;
            }
            continue;
        }

        if (idx < max_len - 1) {
            buf[idx++] = c;
           //uart_poll_out(uart_dev, c);   //echo 
        }
    }
}

/* ============================================================
 * HX711 RAW READ
 * ============================================================ */
int32_t hx711_read_raw(struct hx711_dev *dev)
{
    uint32_t data = 0;

    while (gpio_pin_get_dt(&dev->dt) == 1) {
        k_busy_wait(10);
    }

    unsigned int key = irq_lock();

    for (int i = 0; i < 24; i++) {
        gpio_pin_set_dt(&dev->sck, 1);
        k_busy_wait(1);

        data <<= 1;

        if (gpio_pin_get_dt(&dev->dt)) {
            data |= 1;
        }

        gpio_pin_set_dt(&dev->sck, 0);
        k_busy_wait(1);
    }

    /* 25th pulse — sets next read to GAIN 128 (channel A) */
    gpio_pin_set_dt(&dev->sck, 1);
    k_busy_wait(1);
    gpio_pin_set_dt(&dev->sck, 0);

    irq_unlock(key);

    /* Sign-extend from 24-bit two's complement */
    if (data & 0x800000) {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

/* Returns averaged raw value for the currently selected channel */
static int64_t average(int samples)
{
    int ch = (int)atomic_get(&channel);
    int64_t sum = 0;

    for (int i = 0; i < samples; i++) {
        sum += hx711_read_raw(&hx[ch]);
        k_sleep(K_MSEC(NOTIFY_INTERVAL));
    }

    return sum / samples;
}

/* ============================================================
 * NVS INIT
 * Restores log_idx and session counter on every boot.
 * ============================================================ */
static uint16_t log_idx[4] = {0};

static void nvs_init(void)
{
    struct flash_pages_info info;
    int rc;

    fs.flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    fs.offset       = FIXED_PARTITION_OFFSET(storage_partition);

    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
        LOG_ERR("flash page info err");
        return;
    }

    fs.sector_size  = info.size;
    fs.sector_count = FIXED_PARTITION_SIZE(storage_partition) / fs.sector_size;

    rc = nvs_mount(&fs);
    if (rc) {
        LOG_ERR("NVS mount failed %d", rc);
        return;
    }

    LOG_INF("NVS mounted");

    /* Restore per-channel log indices */
    for (int ch = 0; ch < 4; ch++) {
        uint16_t saved = 0;
        int rd = nvs_read(&fs, log_idx_nvs[ch], &saved, sizeof(saved));
        if (rd == sizeof(saved)) {
            log_idx[ch] = saved;
            LOG_INF("CH%d: restored log_idx=%u", ch, log_idx[ch]);
        } else {
            log_idx[ch] = 0;
        }
    }

    /* Increment and persist session counter */

}

/* ============================================================
 * DATA LOGGING
 *
 * log_write_entry() takes a fresh 3-sample average from every
 * channel independently — does NOT depend on send_thread being
 * active. The logger works autonomously even when no BLE client
 * is connected and no streaming command is running.
 *
 * Log string format (time set):
 *   "S<n>|HH:MM:SS|CH<n>|<mv_per_v>"
 *
 * Log string format (time not set):
 *   "S<n>|T+<ms>|CH<n>|<mv_per_v>"
 * ============================================================ */
static void log_write_entry(void)
{
    char buf[80];
    char ts[16];

    get_time_str(ts, sizeof(ts));

    for (int ch = 0; ch < 4; ch++) {

        if (log_full[ch]) {
            LOG_WRN("CH%d log full", ch);
            continue;
        }

        /* ---- Fresh HX711 reading for this channel ---- */

        /* Discard first sample after channel switch (input settling) */
        hx711_read_raw(&hx[ch]);
        k_sleep(K_MSEC(NOTIFY_INTERVAL));

        int64_t raw = 0;
        for (int i = 0; i < 3; i++) {
            raw += hx711_read_raw(&hx[ch]);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
        }
        raw /= 3;

        k_mutex_lock(&data_mutex, K_FOREVER);
        int64_t offset   = raw - tared[ch];
        float   mv_per_v = ((float)offset * 1000.0f)
                           / (2.0f * HX711_GAIN * HX711_FSR);
        k_mutex_unlock(&data_mutex);

        /* ---- Write to NVS ---- */
        uint16_t nvs_id = log_base[ch] + log_idx[ch];

        snprintf(buf, sizeof(buf),
                 "S%u|%s|CH%d|%.6f",
                 session, ts, ch, (double)mv_per_v);
        
        LOG_INF("LOG CH%d: %s", ch, buf);

        int rc = nvs_write(&fs, nvs_id, buf, strlen(buf) + 1);
        if (rc < 0) {
            LOG_ERR("NVS write fail CH%d id=%u : %d", ch, nvs_id, rc);
            continue;
        }

        log_idx[ch]++;

        if (log_idx[ch] >= log_max[ch]) {
            log_full[ch] = true;
            LOG_WRN("CH%d log full at %u entries", ch, log_idx[ch]);
        }

        /* Persist updated index so it survives a reset */
        nvs_write(&fs, log_idx_nvs[ch], &log_idx[ch], sizeof(log_idx[ch]));

 
    }
}

static void load_read_entry(void)
{
    char buf[80];

    uart_send_str("\r\n===== LOGGER DATA =====\r\n");

    /* Find maximum log count among all channels */
    uint16_t max_logs = 0;

    for (int ch = 0; ch < 4; ch++) {
        if (log_idx[ch] > max_logs) {
            max_logs = log_idx[ch];
        }
    }

    /* Print in order:
       CH0 -> CH1 -> CH2 -> CH3
       for every timestamp/index
    */
    for (uint16_t i = 0; i < max_logs; i++) {

        for (int ch = 0; ch < 4; ch++) {

            /* Skip if this channel has fewer logs */
            if (i >= log_idx[ch]) {
                continue;
            }

            uint16_t nvs_id = log_base[ch] + i;

            int rc = nvs_read(&fs, nvs_id, buf, sizeof(buf));

            if (rc > 0) {

                k_mutex_lock(&data_mutex, K_FOREVER);

                uart_send_str(buf);
                uart_send_str("\r\n");

                k_mutex_unlock(&data_mutex);
            }
        }
    }

    uart_send_str("=======================\r\n");
}

/* ============================================================
 * CALIBRATION
 * ============================================================ */
static void tare(uint8_t ch)
{
    int64_t val = average(3);

    k_mutex_lock(&data_mutex, K_FOREVER);
    tared[ch] = val;
    k_mutex_unlock(&data_mutex);
    uart_send_str("DNE");
    LOG_INF("TARE CH%d = %lld", ch, val);
}

static void untare(uint8_t ch)
{
    k_mutex_lock(&data_mutex, K_FOREVER);
    tared[ch] = 0;
    k_mutex_unlock(&data_mutex);
    uart_send_str("DNE");
    LOG_INF("UNTARE CH%d", ch);
}

/* ============================================================
 * COMMAND PROCESSOR
 *
 *  0x1 = STOP
 *  0x2 = START  (lower 28 bits = channel 0-3)
 *  0x5 = SELECT ALL CHANNELS
 *  0x7 = UNTARE (lower 28 bits = channel 0-3)
 *  0x8 = TARE   (lower 28 bits = channel 0-3)
 * ============================================================ */
static void process_command(uint32_t val)
{
    uint8_t cmd = (uint8_t)(val >> 28);

    switch (cmd) {

    case 0x1:   /* STOP */
        atomic_set(&streaming, 0);
        atomic_set(&sel_all,   0);
        /* Also stop the log timer so logging halts with streaming */
        k_timer_stop(&log_interval_timer);
        atomic_set(&log_write, 0);
        uart_send_str("STP");
        break;

    case 0x2:   /* START — single channel */
    {
        uint32_t ch = val & 0x0FFFFFFF;

        if (ch <= 3) {
            atomic_set(&channel, ch);
            atomic_set(&channel_changed, 1);
        } else {
            atomic_set(&channel, 0);
            atomic_set(&channel_changed, 0);
        }
        atomic_set(&sel_all,   0);
        atomic_set(&streaming, 1);
        k_sem_give(&stream_sem);
        LOG_INF("START CH%d", (int)atomic_get(&channel));
        break;
    }

    case 0x5:   /* SELECT ALL */
        atomic_set(&streaming, 0);
        atomic_set(&sel_all,   1);
        k_sem_give(&stream_sem);
        LOG_INF("ALL");
        break;

    case 0x7:   /* UNTARE */
    {
        uint8_t ch = (uint8_t)(val & 0x0FFFFFFF);

        if (ch <= 3) {
            untare(ch);
        } else {
            uart_send_str("ERR\r\n");
        }
        break;
    }

    case 0x8:   /* TARE */
    {
        uint32_t ch = val & 0x0FFFFFFF;

        if (ch <= 3) {
            atomic_set(&channel, ch);
            atomic_set(&channel_changed, 1);
            tare((uint8_t)ch);
        } else {
            uart_send_str("ERR\r\n");
        }
        break;
    }

    default:
        uart_send_str("ERR: unknown cmd\r\n");
        break;
    }
}

/* ============================================================
 * BLE RX
 * ============================================================ */
static void nus_received(struct bt_conn *conn,
                         const uint8_t *const data,
                         uint16_t len)
{
    if (len < 4) {
        return;
    }

    uint32_t val = (uint8_t)data[0]        |
                  ((uint8_t)data[1] <<  8) |
                  ((uint8_t)data[2] << 16) |
                  ((uint8_t)data[3] << 24);

    process_command(val);
}

/* ============================================================
 * SEND THREAD
 * ============================================================ */
void send_thread(void)
{
    char tx_buf[64];

    k_sem_take(&stream_sem, K_FOREVER);

    while (1) {

        /* ---- ALL-CHANNEL MODE ---- */
        if (atomic_get(&sel_all)) {

            for (int ch = 0; ch < 4; ch++) {
                atomic_set(&channel, ch);

                hx711_read_raw(&hx[ch]);
                k_sleep(K_MSEC(NOTIFY_INTERVAL));

                int64_t raw = average(3);

                k_mutex_lock(&data_mutex, K_FOREVER);
                int64_t offset   = raw - tared[ch];
                float   mv_per_v = ((float)offset * 1000.0f)
                                   / (2.0f * HX711_GAIN * HX711_FSR);

                log_data[ch].uptime_ms = k_uptime_get_32();
                log_data[ch].mv_per_v  = mv_per_v;
                k_mutex_unlock(&data_mutex);

                snprintf(tx_buf, sizeof(tx_buf),
                         "CH%d mV/V:%.6f\r\n", ch, (double)mv_per_v);

                uart_send_str(tx_buf);
                if (atomic_get(&connected)) {
                    bt_nus_send(NULL, tx_buf, strlen(tx_buf));
                }
            }

            atomic_set(&sel_all, 0);

        /* ---- SINGLE-CHANNEL STREAMING MODE ---- */
        } else if (atomic_get(&streaming)) {

            if (atomic_get(&channel_changed)) {
                hx711_read_raw(&hx[atomic_get(&channel)]);
                atomic_set(&channel_changed, 0);
            }

            int     ch    = (int)atomic_get(&channel);
            int64_t raw   = average(3);

            k_mutex_lock(&data_mutex, K_FOREVER);
            int64_t offset   = raw - tared[ch];
            float   mv_per_v = ((float)offset * 1000.0f)
                               / (2.0f * HX711_GAIN * HX711_FSR);

            log_data[ch].uptime_ms = k_uptime_get_32();
            log_data[ch].mv_per_v  = mv_per_v;
            k_mutex_unlock(&data_mutex);

            snprintf(tx_buf, sizeof(tx_buf),
                     "CH%d mV/V:%.6f\r\n", ch, (double)mv_per_v);

            uart_send_str(tx_buf);

            if (atomic_get(&connected)) {
                bt_nus_send(NULL, tx_buf, strlen(tx_buf));
            }

            atomic_set(&streaming, 0);

        } else {
            k_sem_take(&stream_sem, K_FOREVER);
        }
    }
}

/* ============================================================
 * LOG THREAD
 *
 * Woken by:
 *   (a) log_timer_expiry() — periodic RTC-timer-driven write
 *   (b) case 16 in uart_cmd_thread — one-shot read dump
 * ============================================================ */
static void log_thread(void)
{
    while (1) {
        k_sem_take(&log_sem, K_FOREVER);

        if (atomic_get(&log_write)) {
            log_write_entry();

        } else if (atomic_get(&log_read)) {
            load_read_entry();
            atomic_set(&log_read, 0);
        }
    }
}

/* ============================================================
 * UART COMMAND THREAD
 *
 *  UART menu (type number + Enter):
 *   1  = STOP
 *   2  = START CH0    3  = START CH1
 *   4  = START CH2    5  = START CH3
 *   6  = TARE  CH0    7  = TARE  CH1
 *   8  = TARE  CH2    9  = TARE  CH3
 *   10 = UNTARE CH0   11 = UNTARE CH1
 *   12 = UNTARE CH2   13 = UNTARE CH3
 *   14 = SELECT ALL
 *   15 = START LOG    (enter interval in seconds)
 *   16 = STOP LOG     (dump all entries to UART)
 *   17 = READ LOG     (dump all entries to UART)
 *   18 = SET TIME     (enter HH:MM:SS)
 *   0  = SET AVDD     (enter value in mV, e.g. 2150)
 *   Y  = SHOW MENU
 * ============================================================ */
void uart_cmd_thread(void)
{
    char     buf[16];
    uint32_t val;

    uart_send_str("PRESS Y FOR MENU\r\n");

    while (1) {

        uart_read_line(buf, sizeof(buf));

        /* Handle Y/y before atoi() since atoi("Y") == 0 */
        if (buf[0] == 'Y' || buf[0] == 'y') {
            uart_send_str(
                "\r\n=== HX711 Commands ===\r\n"
                " 1 = STOP\r\n"
                " 2 = START CH0    3 = START CH1\r\n"
                " 4 = START CH2    5 = START CH3\r\n"
                " 6 = TARE  CH0    7 = TARE  CH1\r\n"
                " 8 = TARE  CH2    9 = TARE  CH3\r\n"
                "10 = UNTARE CH0  11 = UNTARE CH1\r\n"
                "12 = UNTARE CH2  13 = UNTARE CH3\r\n"
                "14 = SELECT ALL\r\n"
                "15 = START LOG (set interval)\r\n"
                "16 = STOP LOG\r\n"
                "17 = READ LOG\r\n"
                "18 = SET TIME (HH:MM:SS)\r\n"
                " 0 = SET AVDD\r\n"
                "======================\r\n");
            continue;
        }

        val = (uint32_t)atoi(buf);

        switch (val) {

        /* ---- STOP ---- */
        case 1:
            process_command(0x1 << 28);
            uart_send_str("STP\r\n");
            break;

        /* ---- START single channel ---- */
        case 2:
            process_command((0x2 << 28) | 0);
            break;
        case 3:
            process_command((0x2 << 28) | 1);
            break;
        case 4:
            process_command((0x2 << 28) | 2);
            break;
        case 5:
            process_command((0x2 << 28) | 3);
            break;

        /* ---- TARE ---- */
        case 6:
            atomic_set(&streaming, 0);
            process_command((0x8 << 28) | 0);
            break;
        case 7:
            atomic_set(&streaming, 0);
            process_command((0x8 << 28) | 1);
            break;
        case 8:
            atomic_set(&streaming, 0);
            process_command((0x8 << 28) | 2);
            break;
        case 9:
            atomic_set(&streaming, 0);
            process_command((0x8 << 28) | 3);
            break;

        /* ---- UNTARE ---- */
        case 10:
            atomic_set(&streaming, 0);
            process_command((0x7 << 28) | 0);
            break;
        case 11:
            atomic_set(&streaming, 0);
            process_command((0x7 << 28) | 1);
            break;
        case 12:
            atomic_set(&streaming, 0);
            process_command((0x7 << 28) | 2);
            break;
        case 13:
            atomic_set(&streaming, 0);
            process_command((0x7 << 28) | 3);
            break;

        /* ---- SELECT ALL ---- */
        case 14:
            process_command(0x5 << 28);
            break;

        /* ---- START LOG — set interval and arm the RTC timer ---- */
        case 15: {
            char input_buf[16];

            nvs_read(&fs, NVS_ID_TIME, &boot_epoch_ms, sizeof(boot_epoch_ms));

             if (boot_epoch_ms < 0) {

                uart_send_str("ERR");
                break;
             }

            uart_send_str("\r\nInterval (seconds): ");

            if (uart_read_line(input_buf, sizeof(input_buf))) {
                uint16_t new_timer = (uint16_t)atoi(input_buf);

                if (new_timer > 0) {
                    k_mutex_lock(&data_mutex, K_FOREVER);
                    timer = new_timer;
                    k_mutex_unlock(&data_mutex);

                    atomic_set(&log_read,  0);
                    atomic_set(&log_write, 1);

                    /*
                     * (Re)start the RTC-backed log timer.
                     * First arg  = initial delay (first fire)
                     * Second arg = period        (subsequent fires)
                     * Both equal so first write fires exactly one
                     * interval after this command.
                     */
                    k_timer_start(&log_interval_timer,
                                  K_SECONDS(new_timer),
                                  K_SECONDS(new_timer));

                    char msg[72];

                    if (boot_epoch_ms >= 0) {
                        char ts[16];
                        get_time_str(ts, sizeof(ts));
                        snprintf(msg, sizeof(msg),
                                 "\r\nLog started: every %u s | "
                                 "Current time: %s\r\n",
                                 new_timer, ts);
                    } else {
                        snprintf(msg, sizeof(msg),
                                 "\r\nLog started: every %u s | "
                                 "Time not set (use cmd 17)\r\n",
                                 new_timer);
                    }

                    uart_send_str(msg);
                } else {
                    uart_send_str("\r\nERR: interval must be > 0\r\n");
                }
            }
            break;
        }

        /* ---- READ LOG — dump all entries to UART ---- */
        case 16:
                k_timer_stop(&log_interval_timer);
                break;

        case 17: {
            
            atomic_set(&log_write, 0);
            atomic_set(&log_read,  1);
            k_sem_give(&log_sem);
            break;
        }

        /* ----------------------------------------------------------------
         * SET TIME (Option B — UART wall-clock entry)
         *
         * User types current time once after boot. Firmware tracks from
         * that point using k_uptime_get() as an offset, so all subsequent
         * log entries carry a real HH:MM:SS timestamp.
         *
         * Math:
         *   typed_ms      = user-entered time converted to ms-since-midnight
         *   boot_epoch_ms = typed_ms - k_uptime_get()
         *
         *   At any future moment:
         *   wall_ms = boot_epoch_ms + k_uptime_get()
         * ---------------------------------------------------------------- */
        case 18: {
            char input_buf[16];
            uart_send_str("\r\nEnter time (HH:MM:SS): ");

            if (uart_read_line(input_buf, sizeof(input_buf))) {
                int h = 0, m = 0, s = 0;

                /*
                 * sscanf returns number of fields matched.
                 * Accepts "14:30:00" or "9:05:03" etc.
                 */
                int matched = sscanf(input_buf, "%d:%d:%d", &h, &m, &s);

                if (matched == 3 &&
                    h >= 0 && h < 24 &&
                    m >= 0 && m < 60 &&
                    s >= 0 && s < 60) {

                    int64_t typed_ms = (int64_t)(h * 3600 + m * 60 + s) * 1000; //seconds converted to ms
                    boot_epoch_ms    = typed_ms - k_uptime_get();// just counts the current uptime(from the system boot it doesnt need to be called to start the counter) and subtracts it from the typed time to get the epoch

                    nvs_write(&fs, NVS_ID_TIME, &boot_epoch_ms, sizeof(boot_epoch_ms));

                    char msg[48];
                    snprintf(msg, sizeof(msg),
                             "\r\nTime set: %02d:%02d:%02d\r\n", h, m, s);
                    uart_send_str(msg);
                    LOG_INF("Wall clock set: %02d:%02d:%02d", h, m, s);

                } else {
                    uart_send_str(
                        "\r\nERR: use HH:MM:SS format "
                        "(e.g. 14:30:00)\r\n");
                }
            }
            break;
        }

        /* ---- SET AVDD — UART only, never via BLE ---- */
        case 0: {
            char input_buf[16];
            uart_send_str("\r\nAVDD (mV): ");

            if (uart_read_line(input_buf, sizeof(input_buf))) {
                float new_avdd = atof(input_buf);

                if (new_avdd > 1000.0f && new_avdd < 6000.0f) {
                    k_mutex_lock(&data_mutex, K_FOREVER);
                    avdd_mv = new_avdd;
                    k_mutex_unlock(&data_mutex);

                    char msg[48];
                    snprintf(msg, sizeof(msg),
                             "\r\nAVDD set: %.2f mV\r\n", (double)new_avdd);
                    uart_send_str(msg);
                    LOG_INF("AVDD updated: %.2f mV", (double)new_avdd);
                } else {
                    uart_send_str("\r\nERR: range 1000-6000 mV\r\n");
                }
            }
            break;
        }

        default:
            uart_send_str("ERR: unknown cmd (Y for menu)\r\n");
            break;
        }
    }
}

/* ============================================================
 * BLE CONNECTION CALLBACKS
 * ============================================================ */
static void on_connected(struct bt_conn *conn, uint8_t err)
{
    my_conn = bt_conn_ref(conn);
    atomic_set(&connected, 1);
    uart_send_str("BLE CONNECTED\r\n");
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    if (my_conn) {
        bt_conn_unref(my_conn);
        my_conn = NULL;
    }

    atomic_set(&connected, 0);
    uart_send_str("BLE DISCONNECTED\r\n");
}

static struct bt_conn_cb connection_callbacks = {
    .connected    = on_connected,
    .disconnected = disconnected_cb,
};

/* ============================================================
 * MAIN
 * ============================================================ */
int main(void)
{
    int err;

    nvs_init();

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (%d)", err);
        return -1;
    }

    for (int i = 0; i < 4; i++) {
        gpio_pin_configure_dt(&hx[i].dt,  GPIO_INPUT);
        gpio_pin_configure_dt(&hx[i].sck, GPIO_OUTPUT_LOW);
    }

    bt_conn_cb_register(&connection_callbacks);

    bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

    static struct bt_nus_cb nus_cb = {
        .received = nus_received,
    };
    bt_nus_init(&nus_cb);

    /*
     * Pass &log_write to the timer callback via user_data so
     * log_timer_expiry() can check the flag without a raw global.
     */
    k_timer_user_data_set(&log_interval_timer, (void *)&log_write);

    uart_send_str("HX711 BLE + UART READY\r\n");
    uart_send_str("TIP: type 17 to set current time before logging.\r\n");

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}

/* ============================================================
 * THREADS
 * ============================================================ */
K_THREAD_DEFINE(send_id,
                STACKSIZE,
                send_thread,
                NULL, NULL, NULL,
                PRIORITY_SEND,
                0, 0);

K_THREAD_DEFINE(uart_cmd_id,
                STACKSIZE,
                uart_cmd_thread,
                NULL, NULL, NULL,
                PRIORITY_UART,
                0, 0);

K_THREAD_DEFINE(log_id,
                STACKSIZE,
                log_thread,
                NULL, NULL, NULL,
                PRIORITY_LOG,
                0, 0);
