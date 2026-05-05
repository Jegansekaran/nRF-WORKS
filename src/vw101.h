#ifndef VW101_H
#define VW101_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------
 * VW101 — Single-channel vibrating wire acquisition module
 * Protocol : MODBUS RTU over 3.3 V TTL UART
 * Baud     : 9600, 8N1
 * Default slave address: 0x01
 * --------------------------------------------------------------- */

/* Slave address (factory default; change if reconfigured) */
#define VW101_SLAVE_ADDR        0x01

/* MODBUS function codes used */
#define MB_FC_READ_HOLDING      0x03
#define MB_FC_WRITE_SINGLE      0x06

/* Register map */
#define VW101_REG_FREQUENCY     0x0000  /* R  — raw ÷ 10 = Hz          */
#define VW101_REG_TEMPERATURE   0x0001  /* R  — (raw − 500) ÷ 10 = °C  */

/* Special raw return values */
#define VW101_RAW_NOT_READ      0x0000  /* value not yet captured       */
#define VW101_RAW_DISCONNECTED  0xFFFF  /* sensor wire disconnected     */
#define VW101_RAW_BROKEN_WIRE   0xFFF3  /* broken wire / CRC error      */

/* RX frame size for FC03 reading 2 registers:
 *  addr(1) + fc(1) + byte_cnt(1) + data(4) + crc(2) = 9 bytes */
#define VW101_RX_LEN            9

/* TX frame for reading both registers in one shot:
 *  01 03 00 00 00 02 C4 0B */
#define VW101_TX_LEN            8

/* Receive timeout — datasheet says data return time 1-5 s;
 * we wait up to 3 s before declaring a timeout.             */
#define VW101_RX_TIMEOUT_MS     3000

/* ---------------------------------------------------------------
 * Result structure
 * --------------------------------------------------------------- */
typedef struct {
    float frequency_hz;     /* vibrating wire frequency, Hz          */
    float temperature_c;    /* temperature in °C                     */
    bool  freq_valid;       /* false if disconnected / not read      */
    bool  temp_valid;       /* false if NTC disconnected             */
} vw101_data_t;

/* ---------------------------------------------------------------
 * Error codes
 * --------------------------------------------------------------- */
typedef enum {
    VW101_OK            =  0,
    VW101_ERR_UART      = -1,   /* UART send/receive failure          */
    VW101_ERR_TIMEOUT   = -2,   /* no response within timeout         */
    VW101_ERR_CRC       = -3,   /* CRC mismatch in response           */
    VW101_ERR_ADDR      = -4,   /* response slave address mismatch    */
    VW101_ERR_FUNC      = -5,   /* response function code mismatch    */
} vw101_err_t;

/* ---------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------- */

/**
 * @brief  Initialise the VW101 driver (configures the UART device).
 * @param  uart_dev  Pointer to the Zephyr UART device (from DT).
 * @return 0 on success, negative errno on failure.
 */
int vw101_init(const struct device *uart_dev);

/**
 * @brief  Read frequency and temperature from the VW101.
 * @param  out  Pointer to result structure populated on success.
 * @return vw101_err_t code.
 */
vw101_err_t vw101_read(vw101_data_t *out);

#endif /* VW101_H */
