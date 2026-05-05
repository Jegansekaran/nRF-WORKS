/*
 * vw101.c — VW101 vibrating wire module driver
 *
 * MODBUS RTU, FC03, registers 0x0000 (frequency) and 0x0001 (temperature).
 * Uses Zephyr polling UART API — simple, no ISR, no DMA required.
 *
 * Timing note: the VW101 needs up to 5 s to capture a reading
 * (full frequency scan < 5 s per datasheet). Poll no faster than 2 s.
 */

#include "vw101.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(vw101, LOG_LEVEL_DBG);

/* ---------------------------------------------------------------
 * Module-level state
 * --------------------------------------------------------------- */
static const struct device *vw101_uart = NULL;

/* Pre-built read request frame (static — never changes) */
static const uint8_t read_req[VW101_TX_LEN] = {
    VW101_SLAVE_ADDR,       /* 0x01 — slave address             */
    MB_FC_READ_HOLDING,     /* 0x03 — function code             */
    0x00, 0x00,             /* start register: 0x0000           */
    0x00, 0x02,             /* quantity: 2 registers            */
    0xC4, 0x0B              /* CRC16 (pre-calculated)           */
};

/* ---------------------------------------------------------------
 * CRC-16/MODBUS
 * Poly: 0xA001 (reflected 0x8005), Init: 0xFFFF
 * --------------------------------------------------------------- */
static uint16_t crc16_modbus(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ---------------------------------------------------------------
 * UART helpers
 * --------------------------------------------------------------- */

/** Flush any stale bytes sitting in the RX FIFO. */
static void uart_flush_rx(void)
{
    uint8_t dummy;
    while (uart_poll_in(vw101_uart, &dummy) == 0) {
        /* discard */
    }
}

/**
 * Send @p len bytes from @p buf over UART.
 * uart_poll_out blocks until each byte is in the TX FIFO — fine at 9600 baud.
 */
static int uart_send(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(vw101_uart, buf[i]);
    }
    return 0;
}

/**
 * Receive exactly @p len bytes into @p buf within @p timeout_ms milliseconds.
 *
 * Uses busy-polling with k_uptime_get() for timeout tracking.
 * At 9600 baud a single byte takes ~1.04 ms, so polling is fine here.
 */
static int uart_recv(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t received = 0;
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (received < len) {
        if (k_uptime_get() > deadline) {
            LOG_WRN("RX timeout after %u ms (%zu/%zu bytes)",
                    timeout_ms, received, len);
            return -ETIMEDOUT;
        }
        int ret = uart_poll_in(vw101_uart, &buf[received]);
        if (ret == 0) {
            received++;
        } else {
            /* No byte yet — yield briefly so Zephyr scheduler can run */
            k_sleep(K_MSEC(1));
        }
    }
    return 0;
}

/* ---------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------- */

int vw101_init(const struct device *uart_dev)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    vw101_uart = uart_dev;
    LOG_INF("VW101 driver initialised on %s", uart_dev->name);
    return 0;
}

vw101_err_t vw101_read(vw101_data_t *out)
{
    if (!vw101_uart || !out) {
        return VW101_ERR_UART;
    }

    uint8_t rx[VW101_RX_LEN];

    /* --- 1. Flush stale RX data, then send request --- */
    uart_flush_rx();

    LOG_DBG("TX: %02X %02X %02X %02X %02X %02X %02X %02X",
            read_req[0], read_req[1], read_req[2], read_req[3],
            read_req[4], read_req[5], read_req[6], read_req[7]);

    uart_send(read_req, VW101_TX_LEN);

    /* --- 2. Receive response --- */
    int ret = uart_recv(rx, VW101_RX_LEN, VW101_RX_TIMEOUT_MS);
    if (ret == -ETIMEDOUT) {
        return VW101_ERR_TIMEOUT;
    }

    LOG_DBG("RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            rx[0], rx[1], rx[2], rx[3],
            rx[4], rx[5], rx[6], rx[7], rx[8]);

    /* --- 3. Validate slave address --- */
    if (rx[0] != VW101_SLAVE_ADDR) {
        LOG_ERR("Slave addr mismatch: got 0x%02X, expected 0x%02X",
                rx[0], VW101_SLAVE_ADDR);
        return VW101_ERR_ADDR;
    }

    /* --- 4. Validate function code --- */
    if (rx[1] != MB_FC_READ_HOLDING) {
        LOG_ERR("Function code mismatch: 0x%02X", rx[1]);
        return VW101_ERR_FUNC;
    }

    /* --- 5. Verify CRC ---
     *  CRC covers bytes 0..6 (addr + fc + byte_cnt + 4 data bytes).
     *  rx[7] = CRC low, rx[8] = CRC high.
     */
    uint16_t crc_calc = crc16_modbus(rx, VW101_RX_LEN - 2);
    uint16_t crc_recv = (uint16_t)rx[7] | ((uint16_t)rx[8] << 8);

    if (crc_calc != crc_recv) {
        LOG_ERR("CRC mismatch: calc=0x%04X, recv=0x%04X",
                crc_calc, crc_recv);
        return VW101_ERR_CRC;
    }

    /* --- 6. Extract register values (big-endian in MODBUS) --- */
    uint16_t raw_freq = ((uint16_t)rx[3] << 8) | rx[4];
    uint16_t raw_temp = ((uint16_t)rx[5] << 8) | rx[6];

    LOG_DBG("raw_freq=0x%04X (%u)  raw_temp=0x%04X (%u)",
            raw_freq, raw_freq, raw_temp, raw_temp);

    /* --- 7. Check frequency sensor status --- */
    if (raw_freq == VW101_RAW_NOT_READ ||
        raw_freq == VW101_RAW_DISCONNECTED ||
        raw_freq == VW101_RAW_BROKEN_WIRE) {

        LOG_WRN("Frequency sensor error: raw=0x%04X", raw_freq);
        out->freq_valid    = false;
        out->frequency_hz  = 0.0f;
    } else {
        out->freq_valid   = true;
        out->frequency_hz = (float)raw_freq / 10.0f;
    }

    /* --- 8. Decode temperature ---
     * Formula from datasheet: T(°C) = (raw − 500) / 10
     * raw = 0xFFFF means NTC sensor not connected.
     */
    if (raw_temp == VW101_RAW_DISCONNECTED) {
        LOG_WRN("Temperature sensor disconnected");
        out->temp_valid    = false;
        out->temperature_c = 0.0f;
    } else {
        out->temp_valid    = true;
        /* raw is unsigned but result can be negative, cast first */
        out->temperature_c = ((float)(int16_t)raw_temp - 500.0f) / 10.0f;
    }

    return VW101_OK;
}
