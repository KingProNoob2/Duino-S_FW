// STM32F407 hardware abstraction glue for MaraVM, MaraProt, USB CDC, and BL24C256A EEPROM.

#ifndef PLC_HAL_H
#define PLC_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mara_prot.h"
#include "mara_vm.h"
#include <stdbool.h>
#include <stdint.h>

#define PLC_HAL_FW_VERSION       0x00010000U
#define PLC_HAL_MODEL_ID         0x0407U
#define PLC_HAL_CDC_RX_BUF_SIZE  2048U
#define PLC_HAL_EEPROM_HDR_ADDR  0x0000U
#define PLC_HAL_EEPROM_IMG_ADDR  0x0020U
#define PLC_HAL_EEPROM_RET_ADDR  0x7020U
#define PLC_HAL_EEPROM_MAGIC     0x4D464C45U

// Initialize EEPROM, load any stored MFLW program, and initialize MaraVM + MaraProt.
bool PLC_HAL_Init(void);

// Push received USB CDC bytes into the lock-free RX ring buffer (called from ISR).
void PLC_HAL_CdcRxPush(const uint8_t *p_data, uint32_t lenBytes);

// Drain USB CDC RX ring buffer into MaraProt (called from FreeRTOS comm task).
void PLC_HAL_PollProtocol(void);

// Step all active MaraVM threads according to a 5 ms base tick.
void PLC_HAL_StepVmTick(uint32_t tick5msCount);

// Reconfigure PLL for 168 MHz SYSCLK and exact 48 MHz USB OTG FS clock.
void PLC_HAL_ConfigClockUsb168MHz(void);

#ifdef __cplusplus
}
#endif

#endif // PLC_HAL_H
