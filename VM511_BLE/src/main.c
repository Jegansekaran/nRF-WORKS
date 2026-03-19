#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
//#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <bluetooth/services/nus.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

//LOG_MODULE_REGISTER(VM511_BLE, LOG_LEVEL_INF);

/* ---------------- THREAD CONFIG ---------------- */

#define STACKSIZE 2048
#define PRIORITY_CALIB 6
#define PRIORITY_SEND 7
#define NOTIFY_INTERVAL 2000

/* ---------------- NVS IDs ---------------- */

#define EEPROM_TARE_ADDR 1
#define EEPROM_SCALE_ADDR 2
#define EEPROM_FLAG_ADDR 3
#define EEPROM_FLAG_UNIT 4
#define EEPROM_FLAG_INT 5
#define EEPROM_FLAG_DEC 6

#define CAL_DONE 0x55

/* ---------------- VM511 Registers ---------------- */

#define REG_S_FRQ 0x23
#define REG_SYS_STA 0x20
#define REG_SYS_FUN 0x03

/* ---------------- GLOBALS ---------------- */

float K = 0.00035;
float tare = 0;
uint16_t integer;
uint16_t decimal;

static struct nvs_fs fs;

/* ---------------- SEMAPHORES ---------------- */

K_SEM_DEFINE(calib_done_sem, 0, 1);
K_SEM_DEFINE(ble_connected_sem, 0, 1);
K_SEM_DEFINE(init_calib_sem, 0, 1);
K_SEM_DEFINE(load_written, 0, 1);
K_SEM_DEFINE(calib_stop, 0, 1);

/* ---------------- ATOMICS ---------------- */

static atomic_t connected = ATOMIC_INIT(0);
static atomic_t freq_received = ATOMIC_INIT(0);
static atomic_t strain_received = ATOMIC_INIT(0);

/* ---------------- DEVICETREE ---------------- */

#define I2C_NODE DT_NODELABEL(vm511)
static const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(I2C_NODE);

/* ---------------- BLE GLOBALS ---------------- */

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

struct bt_conn *my_conn = NULL;

static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),
    4800,
    4850,
    NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* ---------------- NVS INIT ---------------- */

static void nvs_init(void)
{
    struct flash_pages_info info;

    fs.flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    fs.offset = FIXED_PARTITION_OFFSET(storage_partition);

    flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);

    fs.sector_size = info.size;
    fs.sector_count = FIXED_PARTITION_SIZE(storage_partition) / fs.sector_size;

    nvs_mount(&fs);
}

/* ---------------- I2C HELPERS ---------------- */

static int vm511_read_reg(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];

    k_msleep(50); // Short delay before reading
    while(i2c_write_read_dt(&i2c_dev, &reg, 1, buf, 2)){
        //LOG_WRN("Read reg failed, retrying...");
        k_msleep(100);
    }

    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

static int vm511_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF),
    };

    for (int i = 0; i < 3; i++) {
        if (i2c_write_dt(&i2c_dev, buf, sizeof(buf)) == 0) {
            return 0;
        }
    }

    //LOG_ERR("Write reg failed after retries");
    return -EIO;
}

/* ---------------- SENSOR FUNCTIONS ---------------- */

uint16_t hex_value()
{
    uint16_t raw;

    if (vm511_read_reg(REG_S_FRQ, &raw) < 0)
        return 0;

    uint16_t data;
    
    if(vm511_read_reg(REG_SYS_STA, &data))
        return 0;

    if (data & 0x0020) {
        //LOG_WRN("frequency overflow");
        return false;
    }
    if (data & 0x0008) {
        //LOG_WRN("Low signal quality");
        return false;
    }

    return raw;
}

/* ------------------------------------ 
int64_t average(int samples)
{
    int64_t sum = 0;

    for (int i = 0; i < samples; i++)
        sum += hex_value();

    return sum / samples;
}
---------------------------------------
*/

void offset_value(void)
{
    tare = hex_value()/10.0f;
    int ret;
    ret = nvs_write(&fs, EEPROM_TARE_ADDR, &tare, sizeof(tare));
    if (ret < 0) {
        //LOG_ERR("Failed to write tare value");
    }
}

float final_value(void)
{
    float raw = hex_value()/10.0f;
    nvs_read(&fs, EEPROM_TARE_ADDR, &tare, sizeof(tare));


    if (atomic_get(&freq_received)){
        return raw;
         }

    if (atomic_get(&strain_received)){
        float delta = raw - tare;
        return K * delta * (raw + tare);
        }

    if(!raw){

            bt_nus_send(NULL,"LOW SIGNAL RETRY",16);
        }
}

static float k_value(void)
{
    nvs_read(&fs, EEPROM_FLAG_INT, &integer, sizeof(integer));
    nvs_read(&fs, EEPROM_FLAG_DEC, &decimal, sizeof(decimal));
    if (integer == 0 && decimal == 0) {
        return 0.00035f; // Default K value
    }
    else {
        double result = integer * pow(10, -decimal);
        //LOG_INF("k_value: integer=%u decimal=%u", integer, decimal);
        return (float)result;
        
    }
}


/* ---------------- CALIBRATION ---------------- */
/*
void unlock_calibration(void)
{
    uint8_t f = 0xFF;
    nvs_write(&fs, EEPROM_FLAG_ADDR, &f, sizeof(f));
}

bool load_calibration(void)
{
    uint8_t flag;

    if (nvs_read(&fs, EEPROM_FLAG_ADDR, &flag, sizeof(flag)) <= 0)
        return false;

    if (flag != CAL_DONE)
        return false;

    nvs_read(&fs, EEPROM_TARE_ADDR, &tare, sizeof(tare));

    return true;
}

void save_calibration_once(void)
{
    uint8_t flag = CAL_DONE;

    int ret;
    ret = nvs_write(&fs, EEPROM_TARE_ADDR, &tare, sizeof(tare));
    if (ret < 0) {
        LOG_ERR("Failed to write tare value");
    }
    ret = nvs_write(&fs, EEPROM_FLAG_ADDR, &flag, sizeof(flag));
    if (ret < 0) {
        LOG_ERR("Failed to write calibration flag");
    }
}*/


/* ---------------- BLE COMMAND RECEIVER ---------------- */

static void nus_received(struct bt_conn *conn,
                         const uint8_t *const data,
                         uint16_t len)
{
    if (len < 4)
        return;

    int val = data[0] |
              (data[1] << 8) |
              (data[2] << 16) |
              (data[3] << 24);

    //LOG_INF("RX bytes: %02X %02X %02X %02X", data[3], data[2], data[1], data[0]);
    if ((val >> 28) == 0x0) {
        integer = val;
        nvs_write(&fs, EEPROM_FLAG_INT, &integer, sizeof(integer));
    }

    else if ((val >> 28) == 0x5) {
        decimal = val & 0x0FFFFFFF;
        nvs_write(&fs, EEPROM_FLAG_DEC, &decimal, sizeof(decimal));
    }

    else if ((val >> 28) == 0x1) {
        atomic_set(&freq_received, 1);
        atomic_set(&strain_received, 0);
        k_sem_give(&load_written);
        k_sem_give(&calib_done_sem);
    }

    else if ((val >> 28) == 0x2) {
        atomic_set(&strain_received, 1);
        atomic_set(&freq_received, 0);
        k_sem_give(&load_written);
        k_sem_give(&calib_done_sem);
    }

    else if ((val >> 28) == 0x3) {
        //unlock_calibration();
        k_sem_give(&init_calib_sem);
        k_sem_give(&calib_stop);
        
    }
}

/* ---------------- CALIB THREAD ---------------- */

void calibration_thread(void)
{
    k_sem_take(&ble_connected_sem, K_FOREVER);
    char txt[64];

    while(1){
        
        k_sem_take(&calib_stop, K_FOREVER);
        k_sem_take(&init_calib_sem, K_FOREVER);
        //LOG_INF("Perform tare");

        offset_value();
        K = k_value();
        //save_calibration_once();
        
        snprintf(txt, sizeof(txt), "F0 = %.2fHz",tare);
        bt_nus_send(NULL, txt, strlen(txt));
        //LOG_INF("%s",txt);
        k_msleep(100);
        snprintf(txt, sizeof(txt), "K = %f",K);
        bt_nus_send(NULL, txt, strlen(txt));
        //LOG_INF("%s",txt);
    }
    
    
}

/* ---------------- SEND THREAD ---------------- */

void send_thread(void)
{
    char tx_buf[64];

    k_sem_take(&calib_done_sem, K_FOREVER);

    while (1) {

        k_sem_take(&load_written, K_FOREVER);
        float value = final_value();

        
        if(atomic_get(&freq_received)){
                snprintf(tx_buf, sizeof(tx_buf), "F: %.2f Hz\r\n", value);
                atomic_set(&freq_received, 0);
                }
        else if(atomic_get(&strain_received)){
                snprintf(tx_buf, sizeof(tx_buf), "STRAIN: %.2f\r\n", value);
                atomic_set(&strain_received, 0);
                }

        bt_nus_send(NULL, tx_buf, strlen(tx_buf));

        //LOG_INF("Sent: %s", tx_buf);
    }
}

/* ---------------- BLE CONNECTION ---------------- */

void on_connected(struct bt_conn *conn, uint8_t err)
{
    my_conn = bt_conn_ref(conn);
    atomic_set(&connected, 1);

    //LOG_INF("BLE Connected");

    k_sem_give(&ble_connected_sem);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    if (my_conn) {
        bt_conn_unref(my_conn);
        my_conn = NULL;
    }

    atomic_set(&connected, 0);

    //LOG_INF("BLE Disconnected");
}

struct bt_conn_cb connection_callbacks = {
    .connected = on_connected,
    .disconnected = disconnected_cb,
};

/* ---------------- MAIN ---------------- */

int main(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        //LOG_ERR("BLE init failed");
        return -1;
    }

    bt_conn_cb_register(&connection_callbacks);

    static struct bt_nus_cb nus_cb = {
        .received = nus_received,
    };

    bt_nus_init(&nus_cb);

    bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

    nvs_init();

    if (!device_is_ready(i2c_dev.bus)) {
        //LOG_ERR("I2C not ready");
        return -1;
    }

    // In main():
    vm511_write_reg(0x0A, 0x000D);  // EX_METH
    k_msleep(3);
    vm511_write_reg(0x0F, 300);     // FS_FMIN = 300 Hz
    k_msleep(3);
    vm511_write_reg(0x10, 8000);    // FS_FMAX = 8000 Hz  ← was 0x01 (BAUD)
    k_msleep(3);
    vm511_write_reg(0x1D, 50);      // EXS_TH = 50% (lower quality threshold)
    k_msleep(3);
    // 1. Bigger sweep step → full re-sweep is faster
    vm511_write_reg(0x11, 5);       // FS_STEP: 5 Hz (default) → 20 Hz
    k_msleep(3);
    // 2. Wider tracking window → fewer full re-sweeps triggered
    vm511_write_reg(0x18, 0x1414);   // FSG_TH: ±50 Hz tracking window (was 0x1414)
    k_msleep(3);
    // 3. Faster measurement cycle
    vm511_write_reg(0x06, 500);      // MM_INTE: 500ms (default) → 200ms
    k_msleep(3);
    vm511_write_reg(0x08, 50);       // RD_INTE: 200ms (default) → 50ms
    k_msleep(3);


    //LOG_INF("VM511 BLE Started");

    return 0;
}

/* ---------------- THREADS ---------------- */

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