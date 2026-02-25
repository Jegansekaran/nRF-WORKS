#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <bluetooth/services/nus.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(LOAD_TEST, LOG_LEVEL_INF);
/* ---------------- THREAD CONFIG ---------------- */
#define STACKSIZE 1024
#define PRIORITY_CALIB 6
#define PRIORITY_SEND  7
#define NOTIFY_INTERVAL 500

/* ---------------- GPIO DT ---------------- */
#define DT_DO  DT_NODELABEL(do_pin1)
#define DT_SCK DT_NODELABEL(sck2)

/* ================= NVS IDs (EEPROM LIKE) ================= */
#define EEPROM_OFFSET_ADDR   1
#define EEPROM_SCALE_ADDR    2
#define EEPROM_FLAG_ADDR     3
#define EEPROM_FLAG_UNIT     4
#define EEPROM_FLAG_LOAD     5

#define CAL_DONE 0x55

/* ---------------- GLOBALS ---------------- */
K_SEM_DEFINE(calib_done_sem, 0, 1);
K_SEM_DEFINE(ble_connected_sem, 0, 1);
K_SEM_DEFINE(load_write, 0, 1);
K_SEM_DEFINE(pause, 0, 1);
K_SEM_DEFINE(start_kload, 0, 1);
K_SEM_DEFINE(start_nload, 0, 1);

static atomic_t connected = ATOMIC_INIT(0);
static atomic_t load_received = ATOMIC_INIT(0);
static atomic_t cali_done = ATOMIC_INIT(0);
static atomic_t deci_pnt = ATOMIC_INIT(3);

int32_t offset = 0;
int32_t scale  = 1;
int32_t set_load = 0;

static struct nvs_fs fs;

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

struct bt_conn *my_conn = NULL;



static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONNECTABLE |
	 BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
	4800,  
	4850,
	NULL);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

static const struct gpio_dt_spec dt_pin = GPIO_DT_SPEC_GET(DT_DO, gpios);
static const struct gpio_dt_spec sck_pin = GPIO_DT_SPEC_GET(DT_SCK, gpios);

static void nvs_init(void)
{
    struct flash_pages_info info;

    fs.flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    fs.offset = FIXED_PARTITION_OFFSET(storage_partition);
    

    flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);

    fs.sector_size  = info.size;
    fs.sector_count = FIXED_PARTITION_SIZE(storage_partition)/fs.sector_size;

    nvs_mount(&fs);
}

static void update_phy(struct bt_conn *conn)
{
    
    const struct bt_conn_le_phy_param preferred_phy = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
    };
    bt_conn_le_phy_update(conn, &preferred_phy);
}

int32_t hx711_read_raw(void)
{
    int32_t data = 0;

    while (gpio_pin_get_dt(&dt_pin) == 1) {
        k_usleep(3);
    }

    for (int i = 0; i < 24; i++) {
        gpio_pin_set_dt(&sck_pin, 1);
        k_busy_wait(3);

        data <<= 1;

        gpio_pin_set_dt(&sck_pin, 0);
        k_busy_wait(3);

        if (gpio_pin_get_dt(&dt_pin)) {
            data |= 1;
        }
    }

    gpio_pin_set_dt(&sck_pin, 1);
    k_busy_wait(1);
    gpio_pin_set_dt(&sck_pin, 0);

    if (data & 0x800000) {
        data |= 0xFF000000;
    }

    return data;
}

int64_t average(int samples)
{
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += hx711_read_raw();
    }
    return sum / samples;
}

void offset_value(void)
{
    offset = average(5);
}

void scale_value(int32_t known_weight)
{
    int32_t raw = average(5);
    scale = (raw - offset) / known_weight;
    
}

int32_t final_value(void)
{
    int32_t raw = average(3);
    return (raw - offset) / scale;
}

bool load_calibration(void)
{
    uint8_t flag;

    if (nvs_read(&fs, EEPROM_FLAG_ADDR, &flag, sizeof(flag)) <= 0)
        return false;

    if (flag != CAL_DONE)
        return false;

    nvs_read(&fs, EEPROM_OFFSET_ADDR, &offset, sizeof(offset));
    nvs_read(&fs, EEPROM_SCALE_ADDR, &scale, sizeof(scale));

    if (scale <= 0)
        return false;

    return true;
}

void unlock_calibration(void)
{
    uint8_t f = 0xFF;
    nvs_write(&fs, EEPROM_FLAG_ADDR, &f, sizeof(f));
}

void save_calibration_once(void)
{
    uint8_t flag = CAL_DONE;

    nvs_write(&fs, EEPROM_OFFSET_ADDR, &offset, sizeof(offset));
    nvs_write(&fs, EEPROM_SCALE_ADDR, &scale, sizeof(scale));
    nvs_write(&fs, EEPROM_FLAG_ADDR, &flag, sizeof(flag));

}

bool calibrate(int32_t known_weight)
{
    int32_t raw = average(3);
    int32_t delta = raw - offset;

    if (abs(delta) < 10000)
    {
        
        return false;
    }

    scale = delta / (int32_t)known_weight;
    return true;
}


static void nus_received(struct bt_conn *conn,
                         const uint8_t *const data,
                         uint16_t len)
{
    char buf[16] = {0};

    if (len >= sizeof(buf))
        return;

    memcpy(buf, data, len);

    int val =  buf[0] |
                 (buf[1] << 8) |
                 (buf[2] << 16) |
                 (buf[3] << 24);

    if((val >> 28) == 0){ // Check if the value is negative (assuming it's sent as a signed 32-bit integer)
        if (val >= 100 && val <= 1000) {
        set_load = val;
        nvs_write(&fs, EEPROM_FLAG_LOAD, &set_load, sizeof(set_load));
        k_sem_give(&load_write);
        atomic_set(&cali_done, 0);  
        } 
    }

    else if((val >> 28) == 0x1){
        unlock_calibration();
        k_sem_give(&pause);
        atomic_set(&cali_done, 1);
        atomic_set(&load_received, 0);
        
    }

    else if((val >> 28) == 0x2){
        k_sem_give(&calib_done_sem);
        atomic_set(&load_received, 1);
    }

    else if((val >> 28) == 0x3){
        LOG_INF("Calibration command received N.");
        k_sem_give(&start_nload);
     }
    else if((val >> 28) == 0x4){
        LOG_INF("Calibration command received K.");
        k_sem_give(&start_kload);
    }
    else if((val >> 28) == 0x5){
        atomic_set(&deci_pnt, 1);
         LOG_INF("Calibration command received DEC 1.");
         //k_sem_give(&start_kload);
    }
    else if((val >> 28) == 0x6){
        atomic_set(&deci_pnt, 2);
         LOG_INF("Calibration command received DEC 2.");
         //k_sem_give(&start_kload);
    }
    else if((val >> 28) == 0x7){
        atomic_set(&deci_pnt, 3);
         LOG_INF("Calibration command received DEC 3.");
         //k_sem_give(&start_kload);
    }
    else if((val >> 28) == 0x8){
        ;
    }
    
}


void calibration_thread(void)
{
    k_sem_take(&ble_connected_sem, K_FOREVER);
    while(1){

        if(atomic_get(&cali_done)) {

            if (!load_calibration())
            {
                LOG_INF("Waiting for known load (100-1000g)...");
                    
                k_sem_take(&load_write, K_FOREVER);
                    
                LOG_INF("Remove weight for tare...");
                k_sleep(K_SECONDS(2));

                k_sem_take(&start_nload,K_FOREVER);

                offset_value();
                    
                LOG_INF("Apply known load %d g", set_load);
                k_sleep(K_SECONDS(5));
                
                k_sem_take(&start_kload, K_FOREVER);
                LOG_INF("Calibrating...");

                if (calibrate(set_load)){
                
                       save_calibration_once();
                       atomic_set(&cali_done, 0);
                       k_sem_give(&calib_done_sem);

                       LOG_INF("Calibration done!");
                   }
                }
            }

            else {
                k_sem_take(&pause, K_FOREVER);
                LOG_INF("\n");
            }
    }
}
/* ---------------- SEND THREAD ---------------- */

void send_thread(void)
{
    char tx_buf[64];

    k_sem_take(&calib_done_sem, K_FOREVER);

    while (1){

        if(atomic_get(&load_received)){

            nvs_read(&fs, EEPROM_OFFSET_ADDR, &offset, sizeof(offset));
            nvs_read(&fs, EEPROM_SCALE_ADDR, &scale, sizeof(scale));

            int32_t weight = final_value();
            
            switch(atomic_get(&deci_pnt)){
                case 1:
                    snprintf(tx_buf, sizeof(tx_buf),
                    "Weight: %.1f kg\r\n", ((float)weight)/1000.0f);
                    break;
                case 2:
                    snprintf(tx_buf, sizeof(tx_buf),
                    "Weight: %.2f kg\r\n", ((float)weight)/1000.0f);
                    break;
                case 3:
                    snprintf(tx_buf, sizeof(tx_buf),
                    "Weight: %.3f kg\r\n", ((float)weight)/1000.0f);
                    break;
            }
            
            bt_nus_send(NULL, tx_buf, strlen(tx_buf));

            k_sleep(K_MSEC(NOTIFY_INTERVAL)); 
            LOG_INF("Weight sent: %d g", weight);
            atomic_set(&load_received, 0);
            
            
        }else {
            k_sem_take(&calib_done_sem, K_FOREVER);
        }
    }
}


void on_connected(struct bt_conn *conn, uint8_t err)
{
    my_conn = bt_conn_ref(conn);
	atomic_set(&connected, 1);
    k_sem_give(&ble_connected_sem);
	update_phy(my_conn);	
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
       if (my_conn) {
        // Release our reference — stack can now free the conn object
        bt_conn_unref(my_conn);
        my_conn = NULL;
    }

    atomic_set(&connected, 0);
}

struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = disconnected_cb,
};

int main(void)
{
    int err;

	nvs_init();
	
    err = bt_enable(NULL);
    if (err) {
     
        return -1;
    }

    gpio_pin_configure_dt(&dt_pin, GPIO_INPUT | GPIO_PULL_DOWN);
    gpio_pin_configure_dt(&sck_pin, GPIO_OUTPUT_LOW);
    bt_conn_cb_register(&connection_callbacks);

    bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

    static struct bt_nus_cb nus_cb = {
        .received = nus_received,
    };

    bt_nus_init(&nus_cb);

	/*while (1)
	{
        k_sleep(K_FOREVER);
	}*/

    return -1;
}

K_THREAD_DEFINE(calib_id, STACKSIZE,
                calibration_thread,
                NULL, NULL, NULL,
                PRIORITY_CALIB, 0, 0);

K_THREAD_DEFINE(send_id, STACKSIZE,
                send_thread,
                NULL, NULL, NULL,
                PRIORITY_SEND, 0, 0);