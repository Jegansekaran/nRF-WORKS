#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/logging/log.h>
#include <bluetooth/services/nus.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(hx711_ble, LOG_LEVEL_INF);

/* ============================================================
 * CONFIG
 * ============================================================ */
#define STACKSIZE        1536
#define PRIORITY_UART    5
#define PRIORITY_SEND    7
#define NOTIFY_INTERVAL  500

#define HX711_GAIN       128.0f
#define HX711_FSR        8388607.0f   /* 2^23 - 1 */
#define AVDD_MV_DEFAULT  2150.0f      /* measured AVDD — update if remeasured */

K_SEM_DEFINE(stream_sem, 0, 1);
K_MUTEX_DEFINE(data_mutex);

static atomic_t connected       = ATOMIC_INIT(0);
static atomic_t streaming       = ATOMIC_INIT(0);
static atomic_t channel         = ATOMIC_INIT(0);
static atomic_t channel_changed = ATOMIC_INIT(0);
static atomic_t sel_all         = ATOMIC_INIT(0);
static atomic_t done            = ATOMIC_INIT(0);

/* Protected by data_mutex */
static float   avdd_mv    = AVDD_MV_DEFAULT;
static int64_t tared[4]   = {0};

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

struct bt_conn *my_conn = NULL;

/* ============================================================
 * HX711 GPIO
 * ============================================================ */
struct hx711_dev {
    struct gpio_dt_spec dt;
    struct gpio_dt_spec sck;
};


static struct hx711_dev hx[4] = {
    {
        .dt  = GPIO_DT_SPEC_GET(DT_NODELABEL(dt_0), gpios),
        .sck = GPIO_DT_SPEC_GET(DT_NODELABEL(sck_0), gpios),
    },
    {
        .dt  = GPIO_DT_SPEC_GET(DT_NODELABEL(dt_1), gpios),
        .sck = GPIO_DT_SPEC_GET(DT_NODELABEL(sck_1), gpios),
    },
    {
        .dt  = GPIO_DT_SPEC_GET(DT_NODELABEL(dt_2), gpios),
        .sck = GPIO_DT_SPEC_GET(DT_NODELABEL(sck_2), gpios),
    },
    {
        .dt  = GPIO_DT_SPEC_GET(DT_NODELABEL(dt_3), gpios),
        .sck = GPIO_DT_SPEC_GET(DT_NODELABEL(sck_3), gpios),
    }
};

static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

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

static char uart_read_blocking(void)
{
    uint8_t c;
    while (uart_poll_in(uart_dev, &c) != 0) {
        k_sleep(K_MSEC(1));
    }
    return (char)c;
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
            uart_poll_out(uart_dev, c);   /* echo */
        }
    }
}

/* ============================================================
 * HX711 RAW READ
 * ============================================================ */
int32_t hx711_read_raw(struct hx711_dev *dev)
{
    int32_t data = 0;

    /* Wait for DOUT low — chip ready */
    while (gpio_pin_get_dt(&dev->dt) == 1) {
        k_busy_wait(10);
    }

    for (int i = 0; i < 24; i++) {
        gpio_pin_set_dt(&dev->sck, 1);
        k_busy_wait(1);

        data <<= 1;

        gpio_pin_set_dt(&dev->sck, 0);
        k_busy_wait(1);

        if (gpio_pin_get_dt(&dev->dt)) {
            data |= 1;
        }
    }

    /* 25th pulse — keep gain at 128 (channel A) */
    gpio_pin_set_dt(&dev->sck, 1);
    k_busy_wait(1);
    gpio_pin_set_dt(&dev->sck, 0);

    /* Sign-extend 24-bit two's complement → int32 */
    if (data & 0x800000) {
        data |= 0xFF000000;
    }

    return data;
}

/* Returns averaged raw value for the currently selected channel */
static int64_t average(int samples)
{
    int ch = (int)atomic_get(&channel);
    int64_t sum = 0;

    for (int i = 0; i < samples; i++) {
        sum += hx711_read_raw(&hx[ch]);
        k_sleep(K_MSEC(10));
    }

    return sum / samples;
}

/* ============================================================
 * CALIBRATION
 * ============================================================ */

/* Set tare to current reading */
static void tare(uint8_t ch)
{

    int64_t val = average(5);

    if (ch == 0) {
        k_mutex_lock(&data_mutex, K_FOREVER);
        tared[0] = val;
        k_mutex_unlock(&data_mutex);
        LOG_INF("Tare done. tared=%lld", val);
    }
    if (ch == 1) {
        k_mutex_lock(&data_mutex, K_FOREVER);
        tared[1] = val;
        k_mutex_unlock(&data_mutex);
        LOG_INF("Tare done. tared=%lld", val);
    }
    if (ch == 2) {
        k_mutex_lock(&data_mutex, K_FOREVER);
        tared[2] = val;
        k_mutex_unlock(&data_mutex);
        LOG_INF("Tare done. tared=%lld", val);
    }
    if (ch == 3) {
        k_mutex_lock(&data_mutex, K_FOREVER);
        tared[3] = val;
        k_mutex_unlock(&data_mutex);
        LOG_INF("Tare done. tared=%lld", val);
    }
}

/* Remove tare */
static void untare(uint8_t ch)
{
    k_mutex_lock(&data_mutex, K_FOREVER);
    tared[ch] = 0;
    k_mutex_unlock(&data_mutex);
    LOG_INF("CH%d: Untare done.", ch);
}

/* ============================================================
 * COMMAND PROCESSOR
 * ============================================================ */

/*
 * Commands:
 *   0x1 = STOP
 *   0x2 = START
 *   0x3 = TARE
 *   0x4 = UNTARE
 *   0x5 = SELECT ALL CHANNELS
 *   0x8 = CHANNEL SELECT  (lower 28 bits = channel 0–3)
 *
 * NOTE: SET_AVDD (case '9') is UART-only — handled directly in
 *       uart_cmd_thread, NOT here, because it calls uart_read_line()
 *       which must never be called from BLE context.
 */
static void process_command(uint32_t val)
{
    uint8_t cmd = (uint8_t)(val >> 28);

    switch (cmd) {

    case 0x1:
        atomic_set(&streaming, 0);
        atomic_set(&sel_all, 0);
        LOG_INF("Streaming STOPPED");
        break;

    case 0x2:
        atomic_set(&sel_all, 0);
        atomic_set(&streaming, 1);
        k_sem_give(&stream_sem);
        LOG_INF("Streaming STARTED CH%d", (int)atomic_get(&channel));
        break;

    case 0x5:
        /* SELECT ALL — wake send_thread if it is blocked on stream_sem */
        atomic_set(&streaming, 0);
        atomic_set(&sel_all, 1);
        k_sem_give(&stream_sem);
        LOG_INF("ALL channels selected");
        break;

    case 0x7:
    {
        uint8_t ch = (uint8_t)(val & 0x0FFFFFFF);

        if (ch <= 3) {
            untare(ch);
        } else {
            uart_send_str("ERR:INVALID_CH\r\n");
        }
        break;
    }

    case 0x8:
    {
        uint32_t ch = val & 0x0FFFFFFF;

        if (ch <= 3) {
            atomic_set(&channel, ch);
            atomic_set(&channel_changed, 1);
            tare(ch);
            LOG_INF("Channel -> CH%d", (int)ch);
            
        } else {
            uart_send_str("ERR:INVALID_CH\r\n");
        }
        break;
    }

    default:
        uart_send_str("ERR:UNKNOWN_CMD\r\n");
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

    uint32_t val = (uint8_t)data[0]         |
                  ((uint8_t)data[1] << 8)   |
                  ((uint8_t)data[2] << 16)  |
                  ((uint8_t)data[3] << 24);

    process_command(val);
}

/* ============================================================
 * SEND THREAD
 * ============================================================ */
void send_thread(void)
{
    char tx_buf[64];

    /* Wait for first START / SEL_ALL command */
    k_sem_take(&stream_sem, K_FOREVER);

    while (1) {

        /* ---- ALL-CHANNEL MODE ---- */
        if (atomic_get(&sel_all)) {

            for (int ch = 0; ch < 4; ch++) {
                atomic_set(&channel, ch);

                /* Discard first sample after channel switch (input settling) */
                hx711_read_raw(&hx[ch]);
                k_sleep(K_MSEC(10));

                int64_t raw = average(10);

                k_mutex_lock(&data_mutex, K_FOREVER);
                int64_t offset = raw - tared[ch];
                float   avdd  = avdd_mv;
                k_mutex_unlock(&data_mutex);

                float mv_per_v = ((float)offset * 1000.0f)
                                 / (2.0f * HX711_GAIN * HX711_FSR);
                float vin_mV   = ((float)offset * (avdd / 2.0f))
                                 / (HX711_GAIN * HX711_FSR);

                LOG_INF("CH%d | RAW:%lld | offset:%lld | mV/V:%.6f | Vin:%.4f mV",
                        ch, raw, offset, (double)mv_per_v, (double)vin_mV);
                uart_send_str("\n\r");

                snprintf(tx_buf, sizeof(tx_buf),
                         "CH%d mV/V:%.6f\r\n", ch, (double)mv_per_v);

                if (atomic_get(&connected)) {
                    bt_nus_send(NULL, tx_buf, strlen(tx_buf));
                }
                //uart_send_str(tx_buf);

                /* Brief gap so BLE notifications don't collide */
                k_sleep(K_MSEC(50));
            }

            atomic_set(&sel_all, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            //continue;
        }

        /* ---- SINGLE-CHANNEL STREAMING MODE ---- */
        else if (atomic_get(&streaming)) {

            /* Discard first sample after channel change */
            if (atomic_get(&channel_changed)) {
                hx711_read_raw(&hx[atomic_get(&channel)]);
                atomic_set(&channel_changed, 0);
            }

            int ch      = (int)atomic_get(&channel);
            int64_t raw = average(10);

            k_mutex_lock(&data_mutex, K_FOREVER);
            int64_t offset = raw - tared[ch];
            float   avdd  = avdd_mv;
            k_mutex_unlock(&data_mutex);

            /*
             * mV/V — AVDD cancels ratiometrically, no voltage needed:
             *   mV/V = raw * 1000 / (2 * GAIN * FSR)
             */
            float mv_per_v = ((float)offset * 1000.0f)
                             / (2.0f * HX711_GAIN * HX711_FSR);

            /*
             * Absolute input voltage — needs measured AVDD:
             *   Vin_mV = raw * (AVDD/2) / (GAIN * FSR)
             */
            float vin_mV = ((float)offset * (avdd / 2.0f))
                           / (HX711_GAIN * HX711_FSR);

            LOG_INF("CH%d | RAW:%lld | offset:%lld | mV/V:%.6f | Vin:%.4f mV",
                    ch, raw, offset,
                    (double)mv_per_v,
                    (double)vin_mV);
            uart_send_str("\n\r");

            snprintf(tx_buf, sizeof(tx_buf),
                     "CH%d mV/V:%.6f\r\n", ch, (double)mv_per_v);

            if (atomic_get(&connected)) {
                bt_nus_send(NULL, tx_buf, strlen(tx_buf));
            }

            //
            atomic_set(&streaming, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));

        } else {
            k_sem_take(&stream_sem, K_FOREVER);
        }
    }
}

/* ============================================================
 * UART COMMAND THREAD
 * ============================================================ */
void uart_cmd_thread(void)
{
    char     cmd;
    uint32_t val;

    uart_send_str(
        "PRESS Y FOR MENU\r\n");

    while (1) {

        cmd = uart_read_blocking();

        switch (cmd) {

        case '1':
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            val = (0x1 << 28);
            process_command(val);

            uart_send_str("\r\n");

            uart_send_str(
            "\r\n=== HX711 Commands ===\r\n"
            "1=STOP  2=START\r\n"
            "TARE {3=CH0  4=CH1   5=CH2   6=CH3}\r\n"
            "UNTARE {7=CH0  8=CH1   9=CH2  0=CH3}\r\n"
            "# = SET AVDD\r\n"
            "x = SELECT ALL\r\n"
            "======================\r\n");
            
            atomic_set(&done, 1);
            break;

        case '2':
            val = (0x2 << 28);
            process_command(val);
            break;

        case '3':
            val = (0x8 << 28) | 0;
            atomic_set(&streaming, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            process_command(val);
            break;

        case '4':
            val = (0x8 << 28) | 1;
            atomic_set(&streaming, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            process_command(val);
            break;

        case '5':
            val = (0x8 << 28) | 2;
            atomic_set(&streaming, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            process_command(val);
            break;

        case '6':
            val = (0x8 << 28) | 3;
            atomic_set(&streaming, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            process_command(val);
            break;

        case '7':
            val = (0x7 << 28) | 0;
            atomic_set(&streaming, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            process_command(val);
            break;

        case '8':
            val = (0x7 << 28) | 1;
            atomic_set(&streaming, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            process_command(val);
            break;
        
        case '9':
            val = (0x7 << 28) | 2;
            atomic_set(&streaming, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            process_command(val);
            break;
        
        case '0':
            val = (0x7 << 28) | 3;
            atomic_set(&streaming, 0);
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
            process_command(val);
            break;
        /*
         * SET_AVDD — UART only, never via BLE.
         * uart_read_line() blocks on UART input;
         * calling it from BLE context would deadlock.
         */
        case '#':
        {
            char input_buf[16];
            uart_send_str("\r\nEnter AVDD in mV (e.g. 2150): ");

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
                    uart_send_str("\r\nERR: Value out of range (1000-6000 mV)\r\n");
                }
            }
            break;
        }

        case 'x':
        case 'X':
            val = (0x5 << 28);
            process_command(val);
            break;

        case 'Y':
        case 'y':
        uart_send_str(
            "\r\n=== HX711 Commands ===\r\n"
            "1=STOP  2=START\r\n"
            "TARE {3=CH0  4=CH1   5=CH2   6=CH3}\r\n"
            "UNTARE {7=CH0  8=CH1   9=CH2  0=CH3}\r\n"
            "# = SET AVDD\r\n"
            "x = SELECT ALL\r\n"
            "======================\r\n");
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

    uart_send_str("HX711 BLE + UART READY\r\n");

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