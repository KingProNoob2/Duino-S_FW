// Easy-to-use STM32 HAL driver for the Belling BL24C256A-SFRC
// 256-Kbit (32 KB) I2C Serial EEPROM.
//
// Device characteristics:
//   Capacity:      32,768 bytes (0x0000 .. 0x7FFF)
//   Page size:     64 bytes
//   Address width: 16-bit (MSB first)
//   7-bit address: 0x50 | (A2 << 2) | (A1 << 1) | A0  (0x50 .. 0x57)
//   Write cycle:   <= 5 ms (handled automatically via ACK polling)

#ifndef BL24C256A_H
#define BL24C256A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BL24C256A_SIZE_BYTES       32768U
#define BL24C256A_PAGE_SIZE        64U
#define BL24C256A_BASE_ADDR_7BIT   0x50U
#define BL24C256A_ADDR_PINS_MAX    7U
#define BL24C256A_DEFAULT_TIMEOUT  100U
#define BL24C256A_WRITE_TIMEOUT    25U
#define BL24C256A_POLL_TIMEOUT_MS  5U

typedef enum {
    BL24C256A_OK = 0,
    BL24C256A_ERR_PARAM,
    BL24C256A_ERR_BOUNDS,
    BL24C256A_ERR_I2C,
    BL24C256A_ERR_TIMEOUT
} Bl24c256aStatus_t;

typedef struct {
    I2C_HandleTypeDef *p_hi2c;
    uint16_t           devAddr8Bit;
    uint32_t           i2cTimeout_ms;
} Bl24c256aDev_t;

// Initialize a BL24C256A device handle and verify the chip ACKs on I2C.
// addrPins is the 3-bit hardware address pin strap (A2..A0, 0..7).
Bl24c256aStatus_t BL24C256A_Init(Bl24c256aDev_t *p_dev, I2C_HandleTypeDef *p_hi2c, uint8_t addrPins);

// Poll the EEPROM until its internal write cycle completes or timeout_ms expires.
Bl24c256aStatus_t BL24C256A_WaitReady(const Bl24c256aDev_t *p_dev, uint32_t timeout_ms);

// Read an arbitrary number of bytes starting at any address (0x0000..0x7FFF).
Bl24c256aStatus_t BL24C256A_Read(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint8_t *p_buf, size_t lenBytes);

// Write an arbitrary number of bytes starting at any address.
// Automatically splits writes at 64-byte page boundaries and waits for tWR.
Bl24c256aStatus_t BL24C256A_Write(const Bl24c256aDev_t *p_dev, uint16_t memAddr, const uint8_t *p_data, size_t lenBytes);

// Update-only write: reads each target chunk first and skips I2C write if unchanged.
Bl24c256aStatus_t BL24C256A_Update(const Bl24c256aDev_t *p_dev, uint16_t memAddr, const uint8_t *p_data, size_t lenBytes);

// Fill a region of EEPROM with a constant byte value (e.g. 0xFF to erase).
Bl24c256aStatus_t BL24C256A_Fill(const Bl24c256aDev_t *p_dev, uint16_t memAddr, size_t lenBytes, uint8_t fillVal);

// Erase the entire 32 KB EEPROM (fills with 0xFF).
static inline Bl24c256aStatus_t BL24C256A_EraseAll(const Bl24c256aDev_t *p_dev)
{
    return BL24C256A_Fill(p_dev, 0x0000U, BL24C256A_SIZE_BYTES, 0xFFU);
}

// Convenience typed read/write helpers
Bl24c256aStatus_t BL24C256A_ReadU8(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint8_t *p_out);
Bl24c256aStatus_t BL24C256A_WriteU8(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint8_t val);

Bl24c256aStatus_t BL24C256A_ReadU16(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint16_t *p_out);
Bl24c256aStatus_t BL24C256A_WriteU16(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint16_t val);

Bl24c256aStatus_t BL24C256A_ReadU32(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint32_t *p_out);
Bl24c256aStatus_t BL24C256A_WriteU32(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint32_t val);

Bl24c256aStatus_t BL24C256A_ReadFloat(const Bl24c256aDev_t *p_dev, uint16_t memAddr, float *p_out);
Bl24c256aStatus_t BL24C256A_WriteFloat(const Bl24c256aDev_t *p_dev, uint16_t memAddr, float val);

#define BL24C256A_READ_STRUCT(p_dev, addr, p_struct) \
    BL24C256A_Read((p_dev), (addr), (uint8_t *)(p_struct), sizeof(*(p_struct)))

#define BL24C256A_WRITE_STRUCT(p_dev, addr, p_struct) \
    BL24C256A_Write((p_dev), (addr), (const uint8_t *)(p_struct), sizeof(*(p_struct)))

#define BL24C256A_UPDATE_STRUCT(p_dev, addr, p_struct) \
    BL24C256A_Update((p_dev), (addr), (const uint8_t *)(p_struct), sizeof(*(p_struct)))

#ifdef __cplusplus
}
#endif

#endif // BL24C256A_H
