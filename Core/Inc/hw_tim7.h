// Shared lazy-initialized TIM7 (1 MHz / 1 us) hardware timer dispatcher for
// non-blocking interrupt-driven CH455G and DS18B20 drivers.

#ifndef HW_TIM7_H
#define HW_TIM7_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef uint32_t (*HwTim7StepFn_t)(void);

// Lazily initialize TIM7 at 1 MHz (1 us tick) and enable TIM7_IRQn.
void HW_TIM7_EnsureInit(void);

// Stop TIM7 and release it (called on PLC_HAL_Init / VM program reload).
void HW_TIM7_DeInit(void);

// Schedule a one-shot callback after delayUs microseconds.
void HW_TIM7_ScheduleUs(uint32_t delayUs, HwTim7StepFn_t fn);

// Cancel any pending TIM7 callback.
void HW_TIM7_Cancel(void);

// Return true if a TIM7 ISR sequence is currently in progress.
bool HW_TIM7_IsBusy(void);

// Enable Cortex-M4 DWT->CYCCNT for sub-microsecond timing.
void HW_DWT_EnsureInit(void);

// Return current microsecond timestamp using DWT->CYCCNT (168 MHz).
uint32_t HW_DWT_Micros(void);

// Busy-wait delay in microseconds (only used during rare explicit bus scan).
void HW_DWT_DelayUs(uint32_t us);

// Called from TIM7_IRQHandler in stm32f4xx_it.c.
void HW_TIM7_OnIrq(void);

#ifdef __cplusplus
}
#endif

#endif // HW_TIM7_H
