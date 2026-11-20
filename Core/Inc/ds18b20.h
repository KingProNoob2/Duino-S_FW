// Non-blocking TIM7-ISR 1-Wire driver for addressable DS18B20 temperature sensors.
// Supports configurable DIO pin, 64-bit ROM Search, Match ROM, and Dallas CRC-8.

#ifndef DS18B20_H
#define DS18B20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define DS18B20_MAX_SENSORS      8U
#define DS18B20_CONV_WAIT_MS     760U

typedef struct {
    uint8_t rom[8];
    float   tempC;
    bool    ok;
    bool    active;
} Ds18b20Sensor_t;

// Reset DS18B20 bus state (called on PLC_HAL_Init / VM program reload).
void DS18B20_ResetAll(void);

// Lazily initialize 1-Wire bus on the given GPIO port/pin and scan connected ROMs.
bool DS18B20_EnsureInit(GPIO_TypeDef *p_port, uint16_t pin);

// Perform a 1-Wire Search ROM (0xF0) scan on the given GPIO port/pin.
// Returns number of discovered sensors (up to DS18B20_MAX_SENSORS).
uint8_t DS18B20_ScanBus(GPIO_TypeDef *p_port, uint16_t pin,
                        Ds18b20Sensor_t *p_outSensors, uint8_t maxSensors);

// Advance non-blocking background conversion/read state machine and return cached reading.
// If rom64 == 0, uses busIndex (0..7). If rom64 != 0, matches that 64-bit ROM (little-endian).
bool DS18B20_PollAndRead(GPIO_TypeDef *p_port, uint16_t pin,
                         uint64_t rom64, uint8_t busIndex,
                         uint32_t nowMs, float *p_tempC, bool *p_ok);

#ifdef __cplusplus
}
#endif

#endif // DS18B20_H
