/**
 * @file    bl24c256a.h
 * @brief   Easy-to-use STM32 HAL driver for the Belling BL24C256A-SFRC
 *          256-Kbit (32 KB) I2C Serial EEPROM.
 *
 * Key device characteristics:
 *   - Capacity:        32,768 bytes (0x0000 .. 0x7FFF)
 *   - Page size:       64 bytes
 *   - Address width:   16-bit (MSB first)
 *   - 7-bit I2C addr:  0x50 | (A2 << 2) | (A1 << 1) | A0  (0x50 .. 0x57)
 *   - Write cycle:     <= 5 ms (handled automatically via ACK polling)
 */

#ifndef __BL24C256A_H__
#define __BL24C256A_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define BL24C256A_SIZE_BYTES      32768U
#define BL24C256A_PAGE_SIZE       64U
#define BL24C256A_BASE_ADDR_7BIT  0x50U
#define BL24C256A_DEFAULT_TIMEOUT 100U
#define BL24C256A_WRITE_TIMEOUT   25U   /* Max wait for tWR ACK polling (ms) */

typedef enum {
    BL24C256A_OK = 0,
    BL24C256A_ERR_PARAM,
    BL24C256A_ERR_BOUNDS,
    BL24C256A_ERR_I2C,
    BL24C256A_ERR_TIMEOUT
} bl24c256a_status_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t           dev_addr_8bit; /* 7-bit address << 1 for STM32 HAL */
    uint32_t           i2c_timeout_ms;
} bl24c256a_t;

/**
 * @brief  Initialize a BL24C256A device handle and verify the chip ACKs on I2C.
 * @param  dev         Pointer to handle struct
 * @param  hi2c        STM32 HAL I2C handle (e.g. &hi2c1)
 * @param  addr_pins   3-bit hardware address pin strap (A2..A0, 0..7).
 *                     Use 0 if A2=A1=A0=GND (7-bit address 0x50).
 */
bl24c256a_status_t bl24c256a_init(bl24c256a_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_pins);

/**
 * @brief  Poll the EEPROM until its internal write cycle completes (or timeout).
 */
bl24c256a_status_t bl24c256a_wait_ready(const bl24c256a_t *dev, uint32_t timeout_ms);

/**
 * @brief  Read arbitrary number of bytes starting at any address (0x0000..0x7FFF).
 */
bl24c256a_status_t bl24c256a_read(const bl24c256a_t *dev, uint16_t mem_addr, uint8_t *buf, size_t len);

/**
 * @brief  Write arbitrary number of bytes starting at any address.
 *         Automatically splits writes at 64-byte page boundaries and waits for tWR.
 */
bl24c256a_status_t bl24c256a_write(const bl24c256a_t *dev, uint16_t mem_addr, const uint8_t *data, size_t len);

/**
 * @brief  Update-only write: reads each target chunk first and skips I2C write if unchanged.
 *         Reduces EEPROM wear when prototyping configuration/state saves.
 */
bl24c256a_status_t bl24c256a_update(const bl24c256a_t *dev, uint16_t mem_addr, const uint8_t *data, size_t len);

/**
 * @brief  Fill a region of EEPROM with a constant byte value (e.g. 0xFF to erase).
 */
bl24c256a_status_t bl24c256a_fill(const bl24c256a_t *dev, uint16_t mem_addr, size_t len, uint8_t value);

/**
 * @brief  Erase the entire 32 KB EEPROM (fills with 0xFF).
 */
static inline bl24c256a_status_t bl24c256a_erase_all(const bl24c256a_t *dev)
{
    return bl24c256a_fill(dev, 0x0000U, BL24C256A_SIZE_BYTES, 0xFFU);
}

/* --- Convenience Typed Read/Write Helpers for Prototyping --- */

bl24c256a_status_t bl24c256a_read_u8(const bl24c256a_t *dev, uint16_t mem_addr, uint8_t *out);
bl24c256a_status_t bl24c256a_write_u8(const bl24c256a_t *dev, uint16_t mem_addr, uint8_t val);

bl24c256a_status_t bl24c256a_read_u16(const bl24c256a_t *dev, uint16_t mem_addr, uint16_t *out);
bl24c256a_status_t bl24c256a_write_u16(const bl24c256a_t *dev, uint16_t mem_addr, uint16_t val);

bl24c256a_status_t bl24c256a_read_u32(const bl24c256a_t *dev, uint16_t mem_addr, uint32_t *out);
bl24c256a_status_t bl24c256a_write_u32(const bl24c256a_t *dev, uint16_t mem_addr, uint32_t val);

bl24c256a_status_t bl24c256a_read_float(const bl24c256a_t *dev, uint16_t mem_addr, float *out);
bl24c256a_status_t bl24c256a_write_float(const bl24c256a_t *dev, uint16_t mem_addr, float val);

#define bl24c256a_read_struct(dev_ptr, addr, struct_ptr) \
    bl24c256a_read((dev_ptr), (addr), (uint8_t *)(struct_ptr), sizeof(*(struct_ptr)))

#define bl24c256a_write_struct(dev_ptr, addr, struct_ptr) \
    bl24c256a_write((dev_ptr), (addr), (const uint8_t *)(struct_ptr), sizeof(*(struct_ptr)))

#define bl24c256a_update_struct(dev_ptr, addr, struct_ptr) \
    bl24c256a_update((dev_ptr), (addr), (const uint8_t *)(struct_ptr), sizeof(*(struct_ptr)))

#ifdef __cplusplus
}
#endif

#endif /* __BL24C256A_H__ */
