/*
 * main.c — VW101 vibrating wire sensor readout
 *
 * Reads frequency and temperature every 5 seconds over MODBUS RTU.
 * Logs results via RTT / UART console. Designed for nRF52832.
 *
 * Pin wiring (DK or custom board):
 *   nRF P0.06 (TX) ──► VW101 Pin 3 (RXD)
 *   nRF P0.08 (RX) ◄── VW101 Pin 2 (TXD)
 *   nRF 3.3 V      ──► VW101 Pin 5 (3.3V)
 *   nRF GND        ──► VW101 Pin 1 (G)
 *
 *   VW101 Pin 10 (S+) ──► Vibrating wire sensor coil +
 *   VW101 Pin 11 (S−) ──► Vibrating wire sensor coil −
 *   VW101 Pin 8  (T)  ──► NTC thermistor input +
 *   VW101 Pin 7  (G)  ──► NTC thermistor GND
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include "vw101.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* UART instance — must match the alias/label in your device tree overlay */
#define VW101_UART_NODE DT_ALIAS(vw101_uart)

/* Polling interval — datasheet: measurement cycle < 2 s; recommend ≥ 2 s  */
#define POLL_INTERVAL_MS    5000

int main(void)
{
    LOG_INF("=== VW101 Vibrating Wire Readout ===");

    /* ----- Obtain UART device from device tree ----- */
    const struct device *uart = DEVICE_DT_GET(VW101_UART_NODE);

    int ret = vw101_init(uart);
    if (ret != 0) {
        LOG_ERR("vw101_init failed: %d", ret);
        return ret;
    }

    LOG_INF("Polling every %d ms...", POLL_INTERVAL_MS);

    vw101_data_t sensor = {0};

    while (1) {
        vw101_err_t err = vw101_read(&sensor);

        switch (err) {
        case VW101_OK:
            if (sensor.freq_valid) {
                /* Integer and fractional parts separated — avoids %f
                 * which requires newlib and increases flash size.       */
                int32_t freq_int  = (int32_t)sensor.frequency_hz;
                int32_t freq_frac = (int32_t)((sensor.frequency_hz
                                               - freq_int) * 10);
                LOG_INF("Frequency : %d.%d Hz", freq_int, freq_frac);
            } else {
                LOG_WRN("Frequency : sensor error / disconnected");
            }

            if (sensor.temp_valid) {
                /* temperature_c can be negative */
                int32_t t_int  = (int32_t)sensor.temperature_c;
                int32_t t_frac = (int32_t)((sensor.temperature_c
                                            - (float)t_int) * 10);
                /* Handle negative fractional display */
                if (t_frac < 0) {
                    t_frac = -t_frac;
                }
                LOG_INF("Temperature: %d.%d °C", t_int, t_frac);
            } else {
                LOG_WRN("Temperature: NTC sensor disconnected");
            }
            break;

        case VW101_ERR_TIMEOUT:
            LOG_ERR("Read error: no response (timeout)");
            break;

        case VW101_ERR_CRC:
            LOG_ERR("Read error: CRC mismatch");
            break;

        case VW101_ERR_ADDR:
            LOG_ERR("Read error: unexpected slave address in response");
            break;

        case VW101_ERR_FUNC:
            LOG_ERR("Read error: unexpected function code in response");
            break;

        default:
            LOG_ERR("Read error: unknown (%d)", err);
            break;
        }

        k_sleep(K_MSEC(POLL_INTERVAL_MS));
    }

    return 0;
}
