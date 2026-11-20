/**
 * @file    bl24c256a.c
 * @brief   STM32 HAL driver implementation for BL24C256A-SFRC 32 KB I2C EEPROM.
 */

#include "bl24c256a.h"
#include <string.h>

#if defined(__has_include)
  #if __has_include("cmsis_os2.h")
    #include "cmsis_os2.h"
    #define BL24C256A_HAS_RTOS 1
  #endif
#endif

static void eeprom_delay_1ms(void)
{
#ifdef BL24C256A_HAS_RTOS
    if (osKernelGetState() == osKernelRunning) {
        osDelay(1);
        return;
    }
#endif
    HAL_Delay(1);
}

static bool check_bounds(uint16_t mem_addr, size_t len)
{
    if (len == 0U) {
        return true;
    }
    return ((uint32_t)mem_addr + (uint32_t)len) <= BL24C256A_SIZE_BYTES;
}

bl24c256a_status_t bl24c256a_init(bl24c256a_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_pins)
{
    if (dev == NULL || hi2c == NULL || addr_pins > 7U) {
        return BL24C256A_ERR_PARAM;
    }

    dev->hi2c           = hi2c;
    dev->dev_addr_8bit  = (uint16_t)((BL24C256A_BASE_ADDR_7BIT | (addr_pins & 0x07U)) << 1U);
    dev->i2c_timeout_ms = BL24C256A_DEFAULT_TIMEOUT;

    return bl24c256a_wait_ready(dev, BL24C256A_WRITE_TIMEOUT);
}

bl24c256a_status_t bl24c256a_wait_ready(const bl24c256a_t *dev, uint32_t timeout_ms)
{
    if (dev == NULL || dev->hi2c == NULL) {
        return BL24C256A_ERR_PARAM;
    }

    uint32_t start = HAL_GetTick();
    for (;;) {
        if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->dev_addr_8bit, 1U, 5U) == HAL_OK) {
            return BL24C256A_OK;
        }
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return BL24C256A_ERR_TIMEOUT;
        }
        eeprom_delay_1ms();
    }
}

bl24c256a_status_t bl24c256a_read(const bl24c256a_t *dev, uint16_t mem_addr, uint8_t *buf, size_t len)
{
    if (dev == NULL || dev->hi2c == NULL || (buf == NULL && len > 0U)) {
        return BL24C256A_ERR_PARAM;
    }
    if (!check_bounds(mem_addr, len)) {
        return BL24C256A_ERR_BOUNDS;
    }
    if (len == 0U) {
        return BL24C256A_OK;
    }

    bl24c256a_status_t st = bl24c256a_wait_ready(dev, BL24C256A_WRITE_TIMEOUT);
    if (st != BL24C256A_OK) {
        return st;
    }

    if (HAL_I2C_Mem_Read(dev->hi2c,
                         dev->dev_addr_8bit,
                         mem_addr,
                         I2C_MEMADD_SIZE_16BIT,
                         buf,
                         (uint16_t)len,
                         dev->i2c_timeout_ms) != HAL_OK) {
        return BL24C256A_ERR_I2C;
    }

    return BL24C256A_OK;
}

bl24c256a_status_t bl24c256a_write(const bl24c256a_t *dev, uint16_t mem_addr, const uint8_t *data, size_t len)
{
    if (dev == NULL || dev->hi2c == NULL || (data == NULL && len > 0U)) {
        return BL24C256A_ERR_PARAM;
    }
    if (!check_bounds(mem_addr, len)) {
        return BL24C256A_ERR_BOUNDS;
    }
    if (len == 0U) {
        return BL24C256A_OK;
    }

    size_t offset = 0U;
    while (offset < len) {
        uint16_t cur_addr      = (uint16_t)(mem_addr + offset);
        size_t   page_rem      = BL24C256A_PAGE_SIZE - (cur_addr % BL24C256A_PAGE_SIZE);
        size_t   remaining     = len - offset;
        uint16_t chunk_len     = (uint16_t)((remaining < page_rem) ? remaining : page_rem);

        bl24c256a_status_t st = bl24c256a_wait_ready(dev, BL24C256A_WRITE_TIMEOUT);
        if (st != BL24C256A_OK) {
            return st;
        }

        if (HAL_I2C_Mem_Write(dev->hi2c,
                              dev->dev_addr_8bit,
                              cur_addr,
                              I2C_MEMADD_SIZE_16BIT,
                              (uint8_t *)(uintptr_t)&data[offset],
                              chunk_len,
                              dev->i2c_timeout_ms) != HAL_OK) {
            return BL24C256A_ERR_I2C;
        }

        offset += chunk_len;
    }

    /* Ensure the final page write cycle completes before returning */
    return bl24c256a_wait_ready(dev, BL24C256A_WRITE_TIMEOUT);
}

bl24c256a_status_t bl24c256a_update(const bl24c256a_t *dev, uint16_t mem_addr, const uint8_t *data, size_t len)
{
    if (dev == NULL || dev->hi2c == NULL || (data == NULL && len > 0U)) {
        return BL24C256A_ERR_PARAM;
    }
    if (!check_bounds(mem_addr, len)) {
        return BL24C256A_ERR_BOUNDS;
    }
    if (len == 0U) {
        return BL24C256A_OK;
    }

    uint8_t verify_buf[BL24C256A_PAGE_SIZE];
    size_t offset = 0U;

    while (offset < len) {
        uint16_t cur_addr  = (uint16_t)(mem_addr + offset);
        size_t   page_rem  = BL24C256A_PAGE_SIZE - (cur_addr % BL24C256A_PAGE_SIZE);
        size_t   remaining = len - offset;
        uint16_t chunk_len = (uint16_t)((remaining < page_rem) ? remaining : page_rem);

        bl24c256a_status_t st = bl24c256a_read(dev, cur_addr, verify_buf, chunk_len);
        if (st != BL24C256A_OK) {
            return st;
        }

        if (memcmp(verify_buf, &data[offset], chunk_len) != 0) {
            if (HAL_I2C_Mem_Write(dev->hi2c,
                                  dev->dev_addr_8bit,
                                  cur_addr,
                                  I2C_MEMADD_SIZE_16BIT,
                                  (uint8_t *)(uintptr_t)&data[offset],
                                  chunk_len,
                                  dev->i2c_timeout_ms) != HAL_OK) {
                return BL24C256A_ERR_I2C;
            }
            st = bl24c256a_wait_ready(dev, BL24C256A_WRITE_TIMEOUT);
            if (st != BL24C256A_OK) {
                return st;
            }
        }

        offset += chunk_len;
    }

    return BL24C256A_OK;
}

bl24c256a_status_t bl24c256a_fill(const bl24c256a_t *dev, uint16_t mem_addr, size_t len, uint8_t value)
{
    if (dev == NULL || dev->hi2c == NULL) {
        return BL24C256A_ERR_PARAM;
    }
    if (!check_bounds(mem_addr, len)) {
        return BL24C256A_ERR_BOUNDS;
    }
    if (len == 0U) {
        return BL24C256A_OK;
    }

    uint8_t page_buf[BL24C256A_PAGE_SIZE];
    memset(page_buf, value, sizeof(page_buf));

    size_t offset = 0U;
    while (offset < len) {
        uint16_t cur_addr  = (uint16_t)(mem_addr + offset);
        size_t   page_rem  = BL24C256A_PAGE_SIZE - (cur_addr % BL24C256A_PAGE_SIZE);
        size_t   remaining = len - offset;
        uint16_t chunk_len = (uint16_t)((remaining < page_rem) ? remaining : page_rem);

        bl24c256a_status_t st = bl24c256a_wait_ready(dev, BL24C256A_WRITE_TIMEOUT);
        if (st != BL24C256A_OK) {
            return st;
        }

        if (HAL_I2C_Mem_Write(dev->hi2c,
                              dev->dev_addr_8bit,
                              cur_addr,
                              I2C_MEMADD_SIZE_16BIT,
                              page_buf,
                              chunk_len,
                              dev->i2c_timeout_ms) != HAL_OK) {
            return BL24C256A_ERR_I2C;
        }

        offset += chunk_len;
    }

    return bl24c256a_wait_ready(dev, BL24C256A_WRITE_TIMEOUT);
}

bl24c256a_status_t bl24c256a_read_u8(const bl24c256a_t *dev, uint16_t mem_addr, uint8_t *out)
{
    return bl24c256a_read(dev, mem_addr, out, sizeof(*out));
}

bl24c256a_status_t bl24c256a_write_u8(const bl24c256a_t *dev, uint16_t mem_addr, uint8_t val)
{
    return bl24c256a_write(dev, mem_addr, &val, sizeof(val));
}

bl24c256a_status_t bl24c256a_read_u16(const bl24c256a_t *dev, uint16_t mem_addr, uint16_t *out)
{
    return bl24c256a_read(dev, mem_addr, (uint8_t *)out, sizeof(*out));
}

bl24c256a_status_t bl24c256a_write_u16(const bl24c256a_t *dev, uint16_t mem_addr, uint16_t val)
{
    return bl24c256a_write(dev, mem_addr, (const uint8_t *)&val, sizeof(val));
}

bl24c256a_status_t bl24c256a_read_u32(const bl24c256a_t *dev, uint16_t mem_addr, uint32_t *out)
{
    return bl24c256a_read(dev, mem_addr, (uint8_t *)out, sizeof(*out));
}

bl24c256a_status_t bl24c256a_write_u32(const bl24c256a_t *dev, uint16_t mem_addr, uint32_t val)
{
    return bl24c256a_write(dev, mem_addr, (const uint8_t *)&val, sizeof(val));
}

bl24c256a_status_t bl24c256a_read_float(const bl24c256a_t *dev, uint16_t mem_addr, float *out)
{
    return bl24c256a_read(dev, mem_addr, (uint8_t *)out, sizeof(*out));
}

bl24c256a_status_t bl24c256a_write_float(const bl24c256a_t *dev, uint16_t mem_addr, float val)
{
    return bl24c256a_write(dev, mem_addr, (const uint8_t *)&val, sizeof(val));
}
