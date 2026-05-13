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

/* FIX 1: timer was uint16_t = -1 (wrapped to 65535).
 * Changed to int32_t so the -1 sentinel is meaningful and
 * the "interval not set" check (timer <= 0) works correctly. */
static int32_t  timer    = -1;

static int64_t  tared[4] = {0};

static bool log_full[4] = {false};

/* ============================================================
 * WALL-CLOCK TIME  (Option B — UART-set, no external RTC chip)
 * ============================================================ */
static int64_t boot_epoch_ms = -1;   /* -1 = time not yet set */

/* FIX 3: Add a separate date store.
 * boot_date_str holds "YY-MM-DD\0" set by the user via case 19.
 * get_date_str() returns this string (or "??-??-??" if not set).
 * The previous get_date_str() was a copy-paste of get_time_str()
 * and returned time, not date. */
static char boot_date_str[12] = "";  /* "YY-MM-DD\0" */

static void get_time_str(char *out, size_t len)
{
    int64_t now_ms  = boot_epoch_ms + k_uptime_get();
    int64_t now_sec = (now_ms / 1000) % 86400;   /* wrap at midnight */

    int h = (int)(now_sec / 3600);
    int m = (int)((now_sec % 3600) / 60);
    int s = (int)(now_sec % 60);

    snprintf(out, len, "%02d:%02d:%02d", h, m, s);
}

/* FIX 3 (continued): Proper date string — returns stored date,
 * not a copy of the time calculation. */
static void get_date_str(char *out, size_t len)
{
    if (boot_date_str[0] != '\0') {
        snprintf(out, len, "%s", boot_date_str);
    }
}

/* ============================================================
 * NVS LAYOUT
 *   ID  5          : session counter
 *   ID  6          : boot_epoch_ms (wall-clock anchor)
 *   ID  7          : boot_date_str (YY-MM-DD)
 *   IDs 10-13      : persisted log_idx per channel
 *   IDs 20-999     : CH0 log entries  (980 max)
 *   IDs 1000-1999  : CH1 log entries
 *   IDs 2000-2999  : CH2 log entries
 *   IDs 3000-3999  : CH3 log entries
 * ============================================================ */
static const uint16_t log_base[4]    = {20,   1000, 2000, 3000};
static const uint16_t log_idx_nvs[4] = {10,   11,   12,   13  };
static const uint16_t log_max[4]     = {980,  1000, 1000, 1000};

#define NVS_ID_SESSION   5
#define NVS_ID_TIME      6
#define NVS_ID_DATE      7   /* FIX 3: new ID for date string */

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

    //LOG_INF("NVS mounted");

    /* Restore per-channel log indices */
    for (int ch = 0; ch < 4; ch++) {
        uint16_t saved = 0;
        int rd = nvs_read(&fs, log_idx_nvs[ch], &saved, sizeof(saved));
        if (rd == sizeof(saved)) {
            log_idx[ch] = saved;
            //LOG_INF("CH%d: restored log_idx=%u", ch, log_idx[ch]);
        } else {
            log_idx[ch] = 0;
        }
    }

    /* FIX 2: Increment and persist session counter.
     * The original code had only a comment here with no implementation. */
   /* uint16_t saved_session = 0;
    int rd = nvs_read(&fs, NVS_ID_SESSION, &saved_session, sizeof(saved_session));
    if (rd == sizeof(saved_session)) {
        session = saved_session + 1;
    } else {
        session = 0;
    }
    nvs_write(&fs, NVS_ID_SESSION, &session, sizeof(session));
    //LOG_INF("Session: %u", session);

    // Restore wall-clock anchor (set by user via SET TIME command) 
    nvs_read(&fs, NVS_ID_TIME, &boot_epoch_ms, sizeof(boot_epoch_ms));

    // FIX 3: Restore date string 
    nvs_read(&fs, NVS_ID_DATE, boot_date_str, sizeof(boot_date_str));*/
}

/* ============================================================
 * DATA LOGGING
 *
 * Log string format (time + date set):
 *   "S<n>|YY-MM-DD HH:MM:SS|CH<n>|<mv_per_v>"
 *
 * Log string format (time not set):
 *   "S<n>|??-??-?? T+<uptime_sec>|CH<n>|<mv_per_v>"
 * ============================================================ */
static void log_write_entry(void)
{
    char buf[96];   /* increased from 80 to fit date + time */
    char ts[16];
    char ds[12];

    get_time_str(ts, sizeof(ts));
    get_date_str(ds, sizeof(ds));   /* FIX 3: include date in log entry */

    for (int ch = 0; ch < 4; ch++) {

        if (log_full[ch]) {
            LOG_WRN("CH%d log full", ch);
            continue;
        }

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

        uint16_t nvs_id = log_base[ch] + log_idx[ch];

        if(ch == 0){
            snprintf(buf, sizeof(buf),
                 "%s, %s, %.6f",
                  ds, ts, (double)mv_per_v);
        }else{
            snprintf(buf, sizeof(buf),
                 "%.6f", (double)mv_per_v);
        }

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

        nvs_write(&fs, log_idx_nvs[ch], &log_idx[ch], sizeof(log_idx[ch]));
    }
}

static void load_read_entry(void)
{
    char buf[96];

    uart_send_str("\r\n===== LOGGER DATA =====\r\n");

    uint16_t max_logs = 0;

    for (int ch = 0; ch < 4; ch++) {
        if (log_idx[ch] > max_logs) {
            max_logs = log_idx[ch];
        }
    }

    for (uint16_t i = 0; i < max_logs; i++) {

        for (int ch = 0; ch < 4; ch++) {

            if (i >= log_idx[ch]) {
                continue;
            }

            uint16_t nvs_id = log_base[ch] + i;

            int rc = nvs_read(&fs, nvs_id, buf, sizeof(buf));

            if (rc > 0) {
                k_mutex_lock(&data_mutex, K_FOREVER);
                uart_send_str(buf);
                if(ch<3){
                uart_send_str(", ");
                }
                k_mutex_unlock(&data_mutex);
            }
        }
        uart_send_str("\r\n");
    }

    uart_send_str("=======================\r\n");
}

/* ============================================================
 * NVS CLEAR ALL
 *
 * Wipes every known NVS ID:
 *   - Session counter (ID 5)
 *   - Wall-clock anchor (ID 6)
 *   - Date string (ID 7)
 *   - Per-channel log indices (IDs 10-13)
 *   - All log entries for all four channels
 *
 * Also resets every in-RAM mirror so the running firmware
 * stays consistent without needing a reboot:
 *   - session, log_idx[], log_full[], boot_epoch_ms,
 *     boot_date_str, tared[]
 * ============================================================ */
static void nvs_clear_all(void)
{
    //uart_send_str("\r\nClearing NVS...\r\n");

    /* Stop logging before touching NVS */
    k_timer_stop(&log_interval_timer);
    atomic_set(&log_write, 0);
    atomic_set(&log_read,  0);

    /* --- Metadata IDs --- */
    nvs_delete(&fs, NVS_ID_SESSION);
    nvs_delete(&fs, NVS_ID_TIME);
    nvs_delete(&fs, NVS_ID_DATE);

    /* --- Per-channel log index IDs (10-13) --- */
    for (int ch = 0; ch < 4; ch++) {
        nvs_delete(&fs, log_idx_nvs[ch]);
    }

    /* --- Log entry IDs for every channel --- */
    for (int ch = 0; ch < 4; ch++) {
        for (uint16_t i = 0; i < log_idx[ch]; i++) {
            nvs_delete(&fs, log_base[ch] + i);
        }
    }

    /* --- Reset in-RAM state to match the now-empty NVS --- */
    k_mutex_lock(&data_mutex, K_FOREVER);

    session       = 0;
    boot_epoch_ms = -1;
    boot_date_str[0] = '\0';

    for (int ch = 0; ch < 4; ch++) {
        log_idx[ch]  = 0;
        log_full[ch] = false;
        tared[ch]    = 0;
    }

    k_mutex_unlock(&data_mutex);

    uart_send_str("ACLR\r\n");
    //LOG_INF("NVS cleared by user");
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
    uart_send_str("TDNE");
    //LOG_INF("TARE CH%d = %lld", ch, val);
}

static void untare(uint8_t ch)
{
    k_mutex_lock(&data_mutex, K_FOREVER);
    tared[ch] = 0;
    k_mutex_unlock(&data_mutex);
    uart_send_str("UDNE");
    //LOG_INF("UNTARE CH%d", ch);
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
        k_timer_stop(&log_interval_timer);
        atomic_set(&log_write, 0);
        uart_send_str("REST");
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
        //LOG_INF("START CH%d", (int)atomic_get(&channel));
        break;
    }

    case 0x5:   /* SELECT ALL */
        atomic_set(&streaming, 0);
        atomic_set(&sel_all,   1);
        k_sem_give(&stream_sem);
        //LOG_INF("ALL");
        break;

    case 0x7:   /* UNTARE */
    {
        uint8_t ch = (uint8_t)(val & 0x0FFFFFFF);

        if (ch <= 3) {
            untare(ch);
        } else {
            uart_send_str("UERR\r\n");
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
            uart_send_str("TERR\r\n");
        }
        break;
    }

    default:
        uart_send_str("CERR\r\n");
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
 *   15 = START LOG
 *   16 = STOP LOG
 *   17 = READ LOG     (dump all entries to UART)
 *   18 = SET TIME     (enter HH:MM:SS)
 *   19 = SET DATE     (enter YY:MM:DD)
 *   20 = CLEAR ALL NVS (wipe logs, session, time, date, tare)
 *   21 = SET INTERVAL (enter seconds)
 *   22 = READ DATE/TIME
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
                "15 = START LOG\r\n"
                "16 = STOP LOG\r\n"
                "17 = READ LOG\r\n"
                "18 = SET TIME (HH:MM:SS)\r\n"
                "19 = SET DATE (YY:MM:DD)\r\n"
                "20 = CLEAR ALL NVS\r\n"
                "21 = SET INTERVAL (seconds)\r\n"
                "22 = READ DATE/TIME\r\n"
                " 0 = SET AVDD\r\n"
                "======================\r\n");
            continue;
        }

        val = (uint32_t)atoi(buf);

        switch (val) {

        /* ---- STOP ---- */
        case 1:
            process_command(0x1 << 28);
            //uart_send_str("RES\r\n");
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

        /* ---- START LOG ---- */
        case 15: {
            nvs_read(&fs, NVS_ID_TIME, &boot_epoch_ms, sizeof(boot_epoch_ms));
            nvs_read(&fs, NVS_ID_DATE, boot_date_str, sizeof(boot_date_str));
            nvs_read(&fs, NVS_ID_SESSION, &session, sizeof(session));

            if (boot_epoch_ms < 0) {
                uart_send_str("WERR\r\n");
                break;
            }

            if (timer <= 0) {
                uart_send_str("IERR\r\n");
                break;
            }

            atomic_set(&log_read,  0);
            atomic_set(&log_write, 1);

            k_timer_start(&log_interval_timer,
                          K_SECONDS(timer),
                          K_SECONDS(timer));

            char ts[16];
            get_time_str(ts, sizeof(ts));
            uart_send_str("\r\nLSTA\r\n");
            break;
        }

        /* ---- STOP LOG ---- */
        /* FIX 5: Original case 16 was missing a break, so it fell through
         * into case 17 (READ LOG) every time STOP was typed.
         * Also added atomic_set(&log_write, 0) to actually stop writing. */
        case 16:
            k_timer_stop(&log_interval_timer);
            atomic_set(&log_write, 0);
            uart_send_str("\r\nLSTP\r\n");
            break;   /* FIX 5: break was missing */

        /* ---- READ LOG — dump all entries to UART ---- */
        case 17: {
            atomic_set(&log_write, 0);
            atomic_set(&log_read,  1);
            k_sem_give(&log_sem);
            break;
        }

        /* ---- SET TIME ---- */
        /* FIX 6: Was case 19 in the original. The menu listed "18 = SET TIME"
         * but the code used case 19, so typing 18 always hit default ("ERR").
         * Renumbered to case 18 to match the menu. */
        case 18: {
            char input_buf[16];
            uart_send_str("\r\nWENT:");

            if (uart_read_line(input_buf, sizeof(input_buf))) {
                int h = 0, m = 0, s = 0;

                int tmatched = sscanf(input_buf, "%d:%d:%d", &h, &m, &s);

                if (tmatched == 3 &&
                    h >= 0 && h < 24 &&
                    m >= 0 && m < 60 &&
                    s >= 0 && s < 60) {

                    int64_t typed_ms = (int64_t)(h * 3600 + m * 60 + s) * 1000;
                    boot_epoch_ms    = typed_ms - k_uptime_get();

                    nvs_write(&fs, NVS_ID_TIME, &boot_epoch_ms, sizeof(boot_epoch_ms));

                    /*char msg[48];
                    snprintf(msg, sizeof(msg),
                             "\r\nWSET\r\n");
                    uart_send_str(msg);*/
                    //LOG_INF("Wall clock set: %02d:%02d:%02d", h, m, s);

                } else {
                    uart_send_str(
                        "\r\nWERR\r\n");
                }
            }
            break;
        }

        /* ---- SET DATE ---- */
        /* FIX 7: Was case 20. Renamed to case 19 to fill the gap left by
         * renumbering SET TIME from 19 to 18. Added NVS persist and
         * boot_date_str storage. Added the missing break so it no longer
         * falls through into case 0 (SET AVDD). */
        case 19: {
            char input_buf[16];
            uart_send_str("\r\nDENT:");

            if (uart_read_line(input_buf, sizeof(input_buf))) {
                int y = 0, m = 0, d = 0;
                int dmatched = sscanf(input_buf, "%d:%d:%d", &d, &m, &y);

                if (dmatched == 3 &&
                    y >= 0  && y < 100 &&
                    m > 0   && m <= 12 &&
                    d > 0   && d <= 31) {

                    /* Store formatted date string for log entries */
                    snprintf(boot_date_str, sizeof(boot_date_str),
                             "%02d-%02d-%02d", d, m, y);

                    /* Persist across reboots */
                    nvs_write(&fs, NVS_ID_DATE,
                              boot_date_str, sizeof(boot_date_str));

                    /*char msg[48];
                    snprintf(msg, sizeof(msg),
                             "\r\nDSET\r\n");
                    uart_send_str(msg);*/
                    //LOG_INF("Date set: %s", boot_date_str);

                } else {
                    uart_send_str(
                        "\r\nDERR\r\n");
                }
            }
            break;   /* FIX 7: break was missing — original fell into case 0 */
        }

        /* ---- CLEAR ALL NVS ---- */
        case 20: {
            /* Ask for confirmation before destroying all stored data */
            uart_send_str("\r\nPASS: ");

            char confirm[8];
            if (uart_read_line(confirm, sizeof(confirm)) &&
                strcmp(confirm, "1234") == 0) {
                nvs_clear_all();
            } else {
                uart_send_str("ABRT.\r\n");
            }
            break;
        }

        /* ---- SET INTERVAL ---- */
        case 21: {
            char input_buf[16];
            uart_send_str("\r\nIENT: ");

            if (!uart_read_line(input_buf, sizeof(input_buf))) {
                uart_send_str("\r\nIERR\r\n");
                break;
            }

            int32_t new_timer = (int32_t)atoi(input_buf);

            if (new_timer <= 0) {
                uart_send_str("IERR\r\n");
                break;
            }

            timer = new_timer;

            /*char msg[40];
            snprintf(msg, sizeof(msg), "\r\nISET\r\n");
            uart_send_str(msg);*/
            //LOG_INF("Log interval set: %d s", timer);
            break;
        }

        /* ---- READ DATE/TIME ---- */
        case 22: {
            char ts[16], ds[12], msg[48];

            if (boot_epoch_ms < 0) {
                uart_send_str("\r\nWERR\r\n");
            } else {
                get_time_str(ts, sizeof(ts));
                get_date_str(ds, sizeof(ds));
                snprintf(msg, sizeof(msg),
                         "%s, %s, %d\r\n", ds, ts, timer);
                uart_send_str(msg);
            }
            break;
        }

        /* ---- SET AVDD — UART only, never via BLE ---- */
        case 0: {
            char input_buf[16];
            uart_send_str("\r\nAVDD: ");

            if (uart_read_line(input_buf, sizeof(input_buf))) {
                float new_avdd = atof(input_buf);

                if (new_avdd > 1000.0f && new_avdd < 6000.0f) {
                    k_mutex_lock(&data_mutex, K_FOREVER);
                    avdd_mv = new_avdd;
                    k_mutex_unlock(&data_mutex);

                   /* char msg[48];
                    snprintf(msg, sizeof(msg),
                             "\r\nAVDD set: %.2f mV\r\n", (double)new_avdd);
                    uart_send_str(msg);*/
                    //LOG_INF("AVDD updated: %.2f mV", (double)new_avdd);
                } else {
                    uart_send_str("\r\nVERR\r\n");
                }
            }
            break;
        }

        default:
            uart_send_str("\r\nCERR\r\n");
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
    uart_send_str("\r\nCBLE\r\n");
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    if (my_conn) {
        bt_conn_unref(my_conn);
        my_conn = NULL;
    }

    atomic_set(&connected, 0);
    uart_send_str("\r\nDBLE\r\n");
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
        //LOG_ERR("Bluetooth init failed (%d)", err);
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

    k_timer_user_data_set(&log_interval_timer, (void *)&log_write);

    uart_send_str("\r\nASET\r\n");
    /* FIX 8: Tip now correctly references cmd 18 (SET TIME) and 19 (SET DATE) */
    //uart_send_str("TIP: type 18 to set time, 19 to set date before logging.\r\n");

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
