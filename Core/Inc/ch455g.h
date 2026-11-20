// Non-blocking TIM7-ISR 2-wire driver for WCH CH455G 7-Segment LED Display.
// Supports configurable SCL/SDA GPIO pins and ignores 9th-clock NACK.

#ifndef CH455G_H
#define CH455G_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define CH455G_CMD_SYS        0x48U
#define CH455G_CMD_DIG0       0x68U
#define CH455G_CMD_DIG1       0x6AU
#define CH455G_CMD_DIG2       0x6CU
#define CH455G_CMD_DIG3       0x6EU

// Reset CH455G state (called on PLC_HAL_Init / VM program reload).
void CH455G_ResetAll(void);

// Lazily initialize CH455G on the specified SCL and SDA GPIO pins.
void CH455G_EnsureInit(GPIO_TypeDef *p_sclPort, uint16_t sclPin,
                       GPIO_TypeDef *p_sdaPort, uint16_t sdaPin);

// Update display enable, brightness (1..8), and 4 digit segment bytes (A=bit0 .. DP=bit7).
// Only queues changed registers into the non-blocking TIM7 ISR ring buffer.
void CH455G_Update(bool enable, uint8_t brightness1to8, const uint8_t segBytes[4]);

// Kick the TIM7 ISR transmitter if frames are queued and TIM7 is idle.
void CH455G_PumpQueue(void);

// Encode a character/digit (0..15 hex, -1 blank, -2 minus) + optional DP into 7-seg byte.
uint8_t CH455G_EncodeDigit(int32_t code, bool dp);

#ifdef __cplusplus
}
#endif

#endif // CH455G_H
