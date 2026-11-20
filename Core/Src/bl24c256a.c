// STM32 HAL driver implementation for BL24C256A-SFRC 32 KB I2C EEPROM.

#include "bl24c256a.h"
#include <string.h>

#if defined(__has_include)
  #if __has_include("cmsis_os2.h")
    #include "cmsis_os2.h"
    #define BL24C256A_HAS_RTOS 1
  #endif
#endif

static void delay_1ms(void)
{
#ifdef BL24C256A_HAS_RTOS
    if (osKernelGetState() == osKernelRunning) {
        osDelay(1);
        return;
    }
#endif
    HAL_Delay(1);
}

static bool check_bounds(uint16_t memAddr, size_t lenBytes)
{
    if (lenBytes == 0U) {
        return true;
    }
    return ((uint32_t)memAddr + (uint32_t)lenBytes) <= BL24C256A_SIZE_BYTES;
}

static uint16_t calc_page_chunk(uint16_t curAddr, size_t remainingBytes)
{
    size_t pageRemaining = BL24C256A_PAGE_SIZE - (curAddr % BL24C256A_PAGE_SIZE);
    return (uint16_t)((remainingBytes < pageRemaining) ? remainingBytes : pageRemaining);
}

Bl24c256aStatus_t BL24C256A_Init(Bl24c256aDev_t *p_dev, I2C_HandleTypeDef *p_hi2c, uint8_t addrPins)
{
    if (p_dev == NULL || p_hi2c == NULL || addrPins > BL24C256A_ADDR_PINS_MAX) {
        return BL24C256A_ERR_PARAM;
    }

    p_dev->p_hi2c        = p_hi2c;
    p_dev->devAddr8Bit   = (uint16_t)((BL24C256A_BASE_ADDR_7BIT | (addrPins & BL24C256A_ADDR_PINS_MAX)) << 1U);
    p_dev->i2cTimeout_ms = BL24C256A_DEFAULT_TIMEOUT;

    return BL24C256A_WaitReady(p_dev, BL24C256A_WRITE_TIMEOUT);
}

Bl24c256aStatus_t BL24C256A_WaitReady(const Bl24c256aDev_t *p_dev, uint32_t timeout_ms)
{
    if (p_dev == NULL || p_dev->p_hi2c == NULL) {
        return BL24C256A_ERR_PARAM;
    }

    uint32_t startTick_ms = HAL_GetTick();
    for (;;) {
        if (HAL_I2C_IsDeviceReady(p_dev->p_hi2c, p_dev->devAddr8Bit, 1U, BL24C256A_POLL_TIMEOUT_MS) == HAL_OK) {
            return BL24C256A_OK;
        }
        if ((HAL_GetTick() - startTick_ms) >= timeout_ms) {
            return BL24C256A_ERR_TIMEOUT;
        }
        delay_1ms();
    }
}

Bl24c256aStatus_t BL24C256A_Read(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint8_t *p_buf, size_t lenBytes)
{
    if (p_dev == NULL || p_dev->p_hi2c == NULL || (p_buf == NULL && lenBytes > 0U)) {
        return BL24C256A_ERR_PARAM;
    }
    if (!check_bounds(memAddr, lenBytes)) {
        return BL24C256A_ERR_BOUNDS;
    }
    if (lenBytes == 0U) {
        return BL24C256A_OK;
    }

    Bl24c256aStatus_t status = BL24C256A_WaitReady(p_dev, BL24C256A_WRITE_TIMEOUT);
    if (status != BL24C256A_OK) {
        return status;
    }

    if (HAL_I2C_Mem_Read(p_dev->p_hi2c,
                         p_dev->devAddr8Bit,
                         memAddr,
                         I2C_MEMADD_SIZE_16BIT,
                         p_buf,
                         (uint16_t)lenBytes,
                         p_dev->i2cTimeout_ms) != HAL_OK) {
        return BL24C256A_ERR_I2C;
    }

    return BL24C256A_OK;
}

Bl24c256aStatus_t BL24C256A_Write(const Bl24c256aDev_t *p_dev, uint16_t memAddr, const uint8_t *p_data, size_t lenBytes)
{
    if (p_dev == NULL || p_dev->p_hi2c == NULL || (p_data == NULL && lenBytes > 0U)) {
        return BL24C256A_ERR_PARAM;
    }
    if (!check_bounds(memAddr, lenBytes)) {
        return BL24C256A_ERR_BOUNDS;
    }
    if (lenBytes == 0U) {
        return BL24C256A_OK;
    }

    size_t offset = 0U;
    while (offset < lenBytes) {
        uint16_t curAddr  = (uint16_t)(memAddr + offset);
        uint16_t chunkLen = calc_page_chunk(curAddr, lenBytes - offset);

        Bl24c256aStatus_t status = BL24C256A_WaitReady(p_dev, BL24C256A_WRITE_TIMEOUT);
        if (status != BL24C256A_OK) {
            return status;
        }

        if (HAL_I2C_Mem_Write(p_dev->p_hi2c,
                              p_dev->devAddr8Bit,
                              curAddr,
                              I2C_MEMADD_SIZE_16BIT,
                              (uint8_t *)(uintptr_t)&p_data[offset],
                              chunkLen,
                              p_dev->i2cTimeout_ms) != HAL_OK) {
            delay_1ms();
            if (BL24C256A_WaitReady(p_dev, BL24C256A_WRITE_TIMEOUT) != BL24C256A_OK ||
                HAL_I2C_Mem_Write(p_dev->p_hi2c,
                                  p_dev->devAddr8Bit,
                                  curAddr,
                                  I2C_MEMADD_SIZE_16BIT,
                                  (uint8_t *)(uintptr_t)&p_data[offset],
                                  chunkLen,
                                  p_dev->i2cTimeout_ms) != HAL_OK) {
                return BL24C256A_ERR_I2C;
            }
        }

        offset += chunkLen;
    }

    // Ensure the final page write cycle completes before returning.
    return BL24C256A_WaitReady(p_dev, BL24C256A_WRITE_TIMEOUT);
}

Bl24c256aStatus_t BL24C256A_Update(const Bl24c256aDev_t *p_dev, uint16_t memAddr, const uint8_t *p_data, size_t lenBytes)
{
    if (p_dev == NULL || p_dev->p_hi2c == NULL || (p_data == NULL && lenBytes > 0U)) {
        return BL24C256A_ERR_PARAM;
    }
    if (!check_bounds(memAddr, lenBytes)) {
        return BL24C256A_ERR_BOUNDS;
    }
    if (lenBytes == 0U) {
        return BL24C256A_OK;
    }

    uint8_t verifyBuf[BL24C256A_PAGE_SIZE];
    size_t  offset = 0U;

    while (offset < lenBytes) {
        uint16_t curAddr  = (uint16_t)(memAddr + offset);
        uint16_t chunkLen = calc_page_chunk(curAddr, lenBytes - offset);

        Bl24c256aStatus_t status = BL24C256A_Read(p_dev, curAddr, verifyBuf, chunkLen);
        if (status != BL24C256A_OK) {
            return status;
        }

        if (memcmp(verifyBuf, &p_data[offset], chunkLen) != 0) {
            if (HAL_I2C_Mem_Write(p_dev->p_hi2c,
                                  p_dev->devAddr8Bit,
                                  curAddr,
                                  I2C_MEMADD_SIZE_16BIT,
                                  (uint8_t *)(uintptr_t)&p_data[offset],
                                  chunkLen,
                                  p_dev->i2cTimeout_ms) != HAL_OK) {
                return BL24C256A_ERR_I2C;
            }
            status = BL24C256A_WaitReady(p_dev, BL24C256A_WRITE_TIMEOUT);
            if (status != BL24C256A_OK) {
                return status;
            }
        }

        offset += chunkLen;
    }

    return BL24C256A_OK;
}

Bl24c256aStatus_t BL24C256A_Fill(const Bl24c256aDev_t *p_dev, uint16_t memAddr, size_t lenBytes, uint8_t fillVal)
{
    if (p_dev == NULL || p_dev->p_hi2c == NULL) {
        return BL24C256A_ERR_PARAM;
    }
    if (!check_bounds(memAddr, lenBytes)) {
        return BL24C256A_ERR_BOUNDS;
    }
    if (lenBytes == 0U) {
        return BL24C256A_OK;
    }

    uint8_t pageBuf[BL24C256A_PAGE_SIZE];
    memset(pageBuf, fillVal, sizeof(pageBuf));

    size_t offset = 0U;
    while (offset < lenBytes) {
        uint16_t curAddr  = (uint16_t)(memAddr + offset);
        uint16_t chunkLen = calc_page_chunk(curAddr, lenBytes - offset);

        Bl24c256aStatus_t status = BL24C256A_WaitReady(p_dev, BL24C256A_WRITE_TIMEOUT);
        if (status != BL24C256A_OK) {
            return status;
        }

        if (HAL_I2C_Mem_Write(p_dev->p_hi2c,
                              p_dev->devAddr8Bit,
                              curAddr,
                              I2C_MEMADD_SIZE_16BIT,
                              pageBuf,
                              chunkLen,
                              p_dev->i2cTimeout_ms) != HAL_OK) {
            return BL24C256A_ERR_I2C;
        }

        offset += chunkLen;
    }

    return BL24C256A_WaitReady(p_dev, BL24C256A_WRITE_TIMEOUT);
}

Bl24c256aStatus_t BL24C256A_ReadU8(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint8_t *p_out)
{
    return BL24C256A_Read(p_dev, memAddr, p_out, sizeof(*p_out));
}

Bl24c256aStatus_t BL24C256A_WriteU8(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint8_t val)
{
    return BL24C256A_Write(p_dev, memAddr, &val, sizeof(val));
}

Bl24c256aStatus_t BL24C256A_ReadU16(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint16_t *p_out)
{
    return BL24C256A_Read(p_dev, memAddr, (uint8_t *)p_out, sizeof(*p_out));
}

Bl24c256aStatus_t BL24C256A_WriteU16(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint16_t val)
{
    return BL24C256A_Write(p_dev, memAddr, (const uint8_t *)&val, sizeof(val));
}

Bl24c256aStatus_t BL24C256A_ReadU32(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint32_t *p_out)
{
    return BL24C256A_Read(p_dev, memAddr, (uint8_t *)p_out, sizeof(*p_out));
}

Bl24c256aStatus_t BL24C256A_WriteU32(const Bl24c256aDev_t *p_dev, uint16_t memAddr, uint32_t val)
{
    return BL24C256A_Write(p_dev, memAddr, (const uint8_t *)&val, sizeof(val));
}

Bl24c256aStatus_t BL24C256A_ReadFloat(const Bl24c256aDev_t *p_dev, uint16_t memAddr, float *p_out)
{
    return BL24C256A_Read(p_dev, memAddr, (uint8_t *)p_out, sizeof(*p_out));
}

Bl24c256aStatus_t BL24C256A_WriteFloat(const Bl24c256aDev_t *p_dev, uint16_t memAddr, float val)
{
    return BL24C256A_Write(p_dev, memAddr, (const uint8_t *)&val, sizeof(val));
}
