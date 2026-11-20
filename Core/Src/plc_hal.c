// STM32F407 hardware abstraction glue for MaraVM, MaraProt, USB CDC, and BL24C256A EEPROM.

#include "plc_hal.h"
#include "bl24c256a.h"
#include "cmsis_os.h"
#include "dac.h"
#include "i2c.h"
#include "main.h"
#include "rng.h"
#include "rtc.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define PLC_DIO_OUT_COUNT        16U
#define PLC_PWM_COUNT            8U
#define PLC_ADC_COUNT            4U
#define PLC_DAC_COUNT            2U
#define PLC_PWM_MAX_ARR          999U
#define PLC_CDC_TX_RETRIES       20U
#define PLC_DFU_BKP_MAGIC        0xDEADBEEFU

typedef struct {
    GPIO_TypeDef *p_port;
    uint16_t      pin;
} PlcGpioPin_t;

typedef struct {
    TIM_HandleTypeDef *p_htim;
    uint32_t           channel;
} PlcPwmChannel_t;

typedef struct {
    uint32_t magic;
    uint32_t imageSizeBytes;
    uint32_t imageCrc32;
    uint32_t reserved;
} PlcEepromHeader_t;

extern volatile uint16_t plc_adc_raw[PLC_ADC_COUNT];

static const PlcGpioPin_t kDoPins[PLC_DIO_OUT_COUNT] = {
    {DIO2_GPIO_Port,  DIO2_Pin},
    {DIO3_GPIO_Port,  DIO3_Pin},
    {DIO13_GPIO_Port, DIO13_Pin},
    {DIO15_GPIO_Port, DIO15_Pin},
    {DIO16_GPIO_Port, DIO16_Pin},
    {DIO17_GPIO_Port, DIO17_Pin},
    {DIO18_GPIO_Port, DIO18_Pin},
    {DIO19_GPIO_Port, DIO19_Pin},
    {DIO20_GPIO_Port, DIO20_Pin},
    {DIO21_GPIO_Port, DIO21_Pin},
    {DIO22_GPIO_Port, DIO22_Pin},
    {DIO23_GPIO_Port, DIO23_Pin},
    {DIO24_GPIO_Port, DIO24_Pin},
    {DIO25_GPIO_Port, DIO25_Pin},
    {DIO26_GPIO_Port, DIO26_Pin},
    {DIO27_GPIO_Port, DIO27_Pin}
};

static const PlcPwmChannel_t kPwmChannels[PLC_PWM_COUNT] = {
    {&htim4, TIM_CHANNEL_1},
    {&htim4, TIM_CHANNEL_2},
    {&htim4, TIM_CHANNEL_3},
    {&htim4, TIM_CHANNEL_4},
    {&htim3, TIM_CHANNEL_1},
    {&htim3, TIM_CHANNEL_2},
    {&htim3, TIM_CHANNEL_3},
    {&htim3, TIM_CHANNEL_4}
};

// Place VM workspace and protocol buffers in zero-wait-state 64 KB CCMRAM
__attribute__((section(".ccmram"))) static VmInstance_t   g_plc_hal_vm;
__attribute__((section(".ccmram"))) static ProtInstance_t g_plc_hal_prot;

static Bl24c256aDev_t   g_plc_hal_eeprom;
static bool             g_plc_hal_eepromReady = false;
static uint8_t          g_plc_hal_cdcRxBuf[PLC_HAL_CDC_RX_BUF_SIZE];
static volatile uint16_t g_plc_hal_rxHead = 0U;
static volatile uint16_t g_plc_hal_rxTail = 0U;

static uint8_t hal_di_read(uint16_t channel)
{
    if (channel == 0U || channel == 14U) {
        return (HAL_GPIO_ReadPin(DIO14_GPIO_Port, DIO14_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    }
    return 0U;
}

static void hal_do_write(uint16_t channel, uint8_t val)
{
    if (channel >= PLC_DIO_OUT_COUNT) {
        return;
    }
    GPIO_PinState state = (val != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(kDoPins[channel].p_port, kDoPins[channel].pin, state);
}

static int32_t hal_ai_read(uint16_t channel)
{
    if (channel >= PLC_ADC_COUNT) {
        return 0;
    }
    return (int32_t)plc_adc_raw[channel];
}

static void hal_ao_write(uint16_t channel, int32_t val)
{
    int32_t clamped = val;
    if (clamped < 0)         { clamped = 0; }
    else if (clamped > 4095) { clamped = 4095; }

    if (channel == 0U) {
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)clamped);
    } else if (channel == 1U) {
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)clamped);
    }
}

static uint8_t hal_pwm_set(uint16_t channel, int32_t freq_hz, float dutyPct)
{
    (void)freq_hz;
    if (channel >= PLC_PWM_COUNT) {
        return 0U;
    }
    float clamped = dutyPct;
    if (clamped < 0.0f)        { clamped = 0.0f; }
    else if (clamped > 100.0f) { clamped = 100.0f; }

    uint32_t compare = (uint32_t)((clamped / 100.0f) * (float)PLC_PWM_MAX_ARR);
    __HAL_TIM_SET_COMPARE(kPwmChannels[channel].p_htim, kPwmChannels[channel].channel, compare);
    return (clamped > 0.0f) ? 1U : 0U;
}

static uint32_t hal_time_micros(void)
{
    return HAL_GetTick() * 1000U;
}

static uint32_t hal_rng_next(void)
{
    uint32_t rnd = 0U;
    if (HAL_RNG_GenerateRandomNumber(&hrng, &rnd) != HAL_OK) {
        return HAL_GetTick();
    }
    return rnd;
}

static void hal_retentive_write(uint16_t paramId, uint64_t rawBits)
{
    if (!g_plc_hal_eepromReady) {
        return;
    }
    uint32_t addr = PLC_HAL_EEPROM_RET_ADDR + ((uint32_t)paramId * sizeof(uint64_t));
    if ((addr + sizeof(uint64_t)) <= BL24C256A_SIZE_BYTES) {
        BL24C256A_Update(&g_plc_hal_eeprom, (uint16_t)addr, (const uint8_t *)&rawBits, sizeof(rawBits));
    }
}

static uint64_t hal_retentive_read(uint16_t paramId)
{
    if (!g_plc_hal_eepromReady) {
        return 0U;
    }
    uint32_t addr = PLC_HAL_EEPROM_RET_ADDR + ((uint32_t)paramId * sizeof(uint64_t));
    uint64_t val  = 0U;
    if ((addr + sizeof(uint64_t)) <= BL24C256A_SIZE_BYTES) {
        BL24C256A_Read(&g_plc_hal_eeprom, (uint16_t)addr, (uint8_t *)&val, sizeof(val));
    }
    return val;
}

static bool cdc_tx_send(const uint8_t *p_data, uint16_t lenBytes)
{
    for (uint8_t attempt = 0U; attempt < PLC_CDC_TX_RETRIES; attempt++) {
        uint8_t res = CDC_Transmit_FS((uint8_t *)(uintptr_t)p_data, lenBytes);
        if (res == USBD_OK) {
            return true;
        }
        if (res != USBD_BUSY) {
            return false;
        }
        osDelay(1);
    }
    return false;
}

static bool eeprom_storage_write(uint32_t offset, const uint8_t *p_data, uint16_t lenBytes)
{
    if (!g_plc_hal_eepromReady) {
        return true;
    }
    uint32_t addr = PLC_HAL_EEPROM_IMG_ADDR + offset;
    if ((addr + lenBytes) > PLC_HAL_EEPROM_RET_ADDR) {
        return false;
    }
    return BL24C256A_Write(&g_plc_hal_eeprom, (uint16_t)addr, p_data, lenBytes) == BL24C256A_OK;
}

static bool eeprom_storage_read(uint32_t offset, uint8_t *p_buf, uint16_t lenBytes)
{
    if (!g_plc_hal_eepromReady) {
        return false;
    }
    uint32_t addr = PLC_HAL_EEPROM_IMG_ADDR + offset;
    if ((addr + lenBytes) > PLC_HAL_EEPROM_RET_ADDR) {
        return false;
    }
    return BL24C256A_Read(&g_plc_hal_eeprom, (uint16_t)addr, p_buf, lenBytes) == BL24C256A_OK;
}

static void enter_rom_dfu(void)
{
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, PLC_DFU_BKP_MAGIC);
    osDelay(20);
    NVIC_SystemReset();
}

static void load_saved_program_from_eeprom(void)
{
    if (!g_plc_hal_eepromReady) {
        return;
    }

    PlcEepromHeader_t hdr;
    if (BL24C256A_READ_STRUCT(&g_plc_hal_eeprom, PLC_HAL_EEPROM_HDR_ADDR, &hdr) != BL24C256A_OK) {
        return;
    }
    if (hdr.magic != PLC_HAL_EEPROM_MAGIC ||
        hdr.imageSizeBytes < VM_HEADER_SIZE_BYTES ||
        hdr.imageSizeBytes > VM_MAX_IMAGE_BYTES) {
        return;
    }

    if (BL24C256A_Read(&g_plc_hal_eeprom,
                       PLC_HAL_EEPROM_IMG_ADDR,
                       g_plc_hal_vm.imageBuf,
                       hdr.imageSizeBytes) == BL24C256A_OK) {
        if (VM_LoadImage(&g_plc_hal_vm, g_plc_hal_vm.imageBuf, hdr.imageSizeBytes) == VM_OK) {
            VM_SetState(&g_plc_hal_vm, VM_STATE_RUNNING);
        }
    }
}

bool PLC_HAL_Init(void)
{
    VmHalVtable_t vtable = {
        .di_read         = hal_di_read,
        .do_write        = hal_do_write,
        .ai_read         = hal_ai_read,
        .ao_write        = hal_ao_write,
        .pwm_set         = hal_pwm_set,
        .time_micros     = hal_time_micros,
        .rtc_unix_sec    = NULL,
        .rng_next        = hal_rng_next,
        .retentive_write = hal_retentive_write,
        .retentive_read  = hal_retentive_read,
        .comm_exec       = NULL
    };

    if (VM_Init(&g_plc_hal_vm, &vtable) != VM_OK) {
        return false;
    }

    g_plc_hal_eepromReady = (BL24C256A_Init(&g_plc_hal_eeprom, &hi2c1, 0U) == BL24C256A_OK);
    if (g_plc_hal_eepromReady) {
        load_saved_program_from_eeprom();
    }

    ProtTransport_t transport = {
        .kind          = PROT_TRANSPORT_USB_CDC,
        .maxMtuBytes   = 512U,
        .modelId       = PLC_HAL_MODEL_ID,
        .fwVersion     = PLC_HAL_FW_VERSION,
        .stationId     = PROT_STATION_P2P,
        .tx_send       = cdc_tx_send,
        .get_uptime_ms = HAL_GetTick,
        .storage_write = eeprom_storage_write,
        .storage_read  = eeprom_storage_read,
        .enter_dfu     = enter_rom_dfu
    };

    return PROT_Init(&g_plc_hal_prot, &g_plc_hal_vm, &transport);
}

void PLC_HAL_CdcRxPush(const uint8_t *p_data, uint32_t lenBytes)
{
    if (p_data == NULL) {
        return;
    }
    for (uint32_t i = 0U; i < lenBytes; i++) {
        uint16_t nextHead = (uint16_t)((g_plc_hal_rxHead + 1U) % PLC_HAL_CDC_RX_BUF_SIZE);
        if (nextHead != g_plc_hal_rxTail) {
            g_plc_hal_cdcRxBuf[g_plc_hal_rxHead] = p_data[i];
            g_plc_hal_rxHead = nextHead;
        }
    }
}

void PLC_HAL_PollProtocol(void)
{
    bool prevUpload = g_plc_hal_prot.uploadActive;

    while (g_plc_hal_rxTail != g_plc_hal_rxHead) {
        uint8_t byte = g_plc_hal_cdcRxBuf[g_plc_hal_rxTail];
        g_plc_hal_rxTail = (uint16_t)((g_plc_hal_rxTail + 1U) % PLC_HAL_CDC_RX_BUF_SIZE);
        PROT_FeedByte(&g_plc_hal_prot, byte);
    }

    // If an upload just committed successfully, persist the EEPROM header
    if (prevUpload && !g_plc_hal_prot.uploadActive && g_plc_hal_vm.hasValidImage && g_plc_hal_eepromReady) {
        PlcEepromHeader_t hdr = {
            .magic          = PLC_HAL_EEPROM_MAGIC,
            .imageSizeBytes = g_plc_hal_vm.imageSizeBytes,
            .imageCrc32     = g_plc_hal_vm.header.payloadCrc32,
            .reserved       = 0U
        };
        BL24C256A_UPDATE_STRUCT(&g_plc_hal_eeprom, PLC_HAL_EEPROM_HDR_ADDR, &hdr);
    }
}

void PLC_HAL_StepVmTick(uint32_t tick5msCount)
{
    if (VM_GetState(&g_plc_hal_vm) != VM_STATE_RUNNING) {
        return;
    }

    // Thread 4: 5 ms
    VM_StepThread(&g_plc_hal_vm, 4U, 5000U);

    // Thread 3: 20 ms
    if ((tick5msCount % 4U) == 0U) {
        VM_StepThread(&g_plc_hal_vm, 3U, 20000U);
    }

    // Thread 2: 100 ms
    if ((tick5msCount % 20U) == 0U) {
        VM_StepThread(&g_plc_hal_vm, 2U, 100000U);
    }

    // Thread 1: 1000 ms
    if ((tick5msCount % 200U) == 0U) {
        VM_StepThread(&g_plc_hal_vm, 1U, 1000000U);
    }

    // Thread 5: Idle
    VM_StepThread(&g_plc_hal_vm, 5U, 5000U);
}

void PLC_HAL_ConfigClockUsb168MHz(void)
{
    RCC_OscInitTypeDef rccOsc = {0};
    RCC_ClkInitTypeDef rccClk = {0};

    // Switch SYSCLK temporarily to HSE before reconfiguring main PLL
    rccClk.ClockType      = RCC_CLOCKTYPE_SYSCLK;
    rccClk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSE;
    HAL_RCC_ClockConfig(&rccClk, FLASH_LATENCY_0);

    rccOsc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    rccOsc.HSEState       = RCC_HSE_ON;
    rccOsc.PLL.PLLState   = RCC_PLL_ON;
    rccOsc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    rccOsc.PLL.PLLM       = 25U;
    rccOsc.PLL.PLLN       = 336U;
    rccOsc.PLL.PLLP       = RCC_PLLP_DIV2;
    rccOsc.PLL.PLLQ       = 7U;
    if (HAL_RCC_OscConfig(&rccOsc) != HAL_OK) {
        Error_Handler();
    }

    rccClk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                            RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rccClk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    rccClk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    rccClk.APB1CLKDivider = RCC_HCLK_DIV4;
    rccClk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&rccClk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}
