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

#define PLC_DIO_MAX_INDEX        28U
#define PLC_PWM_COUNT            8U
#define PLC_ADC_COUNT            4U
#define PLC_DAC_COUNT            2U
#define PLC_PWM_TIM_CLK_HZ       1000000U
#define PLC_PWM_DEFAULT_ARR      999U
#define PLC_CDC_TX_RETRIES       20U
#define PLC_DFU_BKP_MAGIC        0xDEADBEEFU

typedef enum {
    DIO_MODE_UNCONFIGURED = 0,
    DIO_MODE_INPUT        = 1,
    DIO_MODE_OUTPUT       = 2,
    DIO_MODE_PWM          = 3
} PlcDioMode_t;

typedef struct {
    GPIO_TypeDef      *p_port;
    uint16_t           pin;
    TIM_HandleTypeDef *p_htim;
    uint32_t           timChannel;
    uint8_t            gpioAf;
} PlcDioDesc_t;

typedef struct {
    uint32_t magic;
    uint32_t imageSizeBytes;
    uint32_t imageCrc32;
    uint32_t reserved;
} PlcEepromHeader_t;

extern volatile uint16_t plc_adc_raw[PLC_ADC_COUNT];

// Indexed directly by logical DIO channel number (DIO2 .. DIO27).
// Entries 0, 1, and 8 are unmapped.
static const PlcDioDesc_t kDioMap[PLC_DIO_MAX_INDEX] = {
    [0]  = {NULL,           0U,       NULL,   0U,            0U},
    [1]  = {NULL,           0U,       NULL,   0U,            0U},
    [2]  = {DIO2_GPIO_Port,  DIO2_Pin,  NULL,   0U,            0U},
    [3]  = {DIO3_GPIO_Port,  DIO3_Pin,  NULL,   0U,            0U},
    [4]  = {DIO4_GPIO_Port,  DIO4_Pin,  &htim4, TIM_CHANNEL_1, GPIO_AF2_TIM4},
    [5]  = {DIO5_GPIO_Port,  DIO5_Pin,  &htim4, TIM_CHANNEL_2, GPIO_AF2_TIM4},
    [6]  = {DIO6_GPIO_Port,  DIO6_Pin,  &htim4, TIM_CHANNEL_3, GPIO_AF2_TIM4},
    [7]  = {DIO7_GPIO_Port,  DIO7_Pin,  &htim4, TIM_CHANNEL_4, GPIO_AF2_TIM4},
    [8]  = {NULL,           0U,       NULL,   0U,            0U},
    [9]  = {DIO9_GPIO_Port,  DIO9_Pin,  &htim3, TIM_CHANNEL_1, GPIO_AF2_TIM3},
    [10] = {DIO10_GPIO_Port, DIO10_Pin, &htim3, TIM_CHANNEL_2, GPIO_AF2_TIM3},
    [11] = {DIO11_GPIO_Port, DIO11_Pin, &htim3, TIM_CHANNEL_3, GPIO_AF2_TIM3},
    [12] = {DIO12_GPIO_Port, DIO12_Pin, &htim3, TIM_CHANNEL_4, GPIO_AF2_TIM3},
    [13] = {DIO13_GPIO_Port, DIO13_Pin, NULL,   0U,            0U},
    [14] = {DIO14_GPIO_Port, DIO14_Pin, NULL,   0U,            0U},
    [15] = {DIO15_GPIO_Port, DIO15_Pin, NULL,   0U,            0U},
    [16] = {DIO16_GPIO_Port, DIO16_Pin, NULL,   0U,            0U},
    [17] = {DIO17_GPIO_Port, DIO17_Pin, NULL,   0U,            0U},
    [18] = {DIO18_GPIO_Port, DIO18_Pin, NULL,   0U,            0U},
    [19] = {DIO19_GPIO_Port, DIO19_Pin, NULL,   0U,            0U},
    [20] = {DIO20_GPIO_Port, DIO20_Pin, NULL,   0U,            0U},
    [21] = {DIO21_GPIO_Port, DIO21_Pin, NULL,   0U,            0U},
    [22] = {DIO22_GPIO_Port, DIO22_Pin, NULL,   0U,            0U},
    [23] = {DIO23_GPIO_Port, DIO23_Pin, NULL,   0U,            0U},
    [24] = {DIO24_GPIO_Port, DIO24_Pin, NULL,   0U,            0U},
    [25] = {DIO25_GPIO_Port, DIO25_Pin, NULL,   0U,            0U},
    [26] = {DIO26_GPIO_Port, DIO26_Pin, NULL,   0U,            0U},
    [27] = {DIO27_GPIO_Port, DIO27_Pin, NULL,   0U,            0U}
};

// Maps logical PWM index (0..7) to its DIO pin when referenced as PWM0..PWM7
static const uint8_t kPwmIndexToDio[PLC_PWM_COUNT] = {
    4U, 5U, 6U, 7U, 9U, 10U, 11U, 12U
};

// Place VM workspace and protocol buffers in zero-wait-state 64 KB CCMRAM
__attribute__((section(".ccmram"))) static VmInstance_t   g_plc_hal_vm;
__attribute__((section(".ccmram"))) static ProtInstance_t g_plc_hal_prot;

static PlcDioMode_t      g_plc_hal_dioMode[PLC_DIO_MAX_INDEX];
static Bl24c256aDev_t    g_plc_hal_eeprom;
static bool              g_plc_hal_eepromReady = false;
static uint8_t           g_plc_hal_cdcRxBuf[PLC_HAL_CDC_RX_BUF_SIZE];
static volatile uint16_t g_plc_hal_rxHead = 0U;
static volatile uint16_t g_plc_hal_rxTail = 0U;

static void ensure_dio_mode(uint16_t dioNum, PlcDioMode_t targetMode)
{
    if (dioNum >= PLC_DIO_MAX_INDEX || kDioMap[dioNum].p_port == NULL) {
        return;
    }
    if (g_plc_hal_dioMode[dioNum] == targetMode) {
        return;
    }

    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin  = kDioMap[dioNum].pin;
    gpioInit.Pull = GPIO_NOPULL;

    if (targetMode == DIO_MODE_INPUT) {
        gpioInit.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(kDioMap[dioNum].p_port, &gpioInit);
    } else if (targetMode == DIO_MODE_OUTPUT) {
        gpioInit.Mode  = GPIO_MODE_OUTPUT_PP;
        gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(kDioMap[dioNum].p_port, &gpioInit);
    } else if (targetMode == DIO_MODE_PWM && kDioMap[dioNum].p_htim != NULL) {
        gpioInit.Mode      = GPIO_MODE_AF_PP;
        gpioInit.Speed     = GPIO_SPEED_FREQ_HIGH;
        gpioInit.Alternate = kDioMap[dioNum].gpioAf;
        HAL_GPIO_Init(kDioMap[dioNum].p_port, &gpioInit);
    }

    g_plc_hal_dioMode[dioNum] = targetMode;
}

static uint8_t hal_di_read(uint16_t channel)
{
    if (channel >= PLC_DIO_MAX_INDEX || kDioMap[channel].p_port == NULL) {
        return 0U;
    }
    ensure_dio_mode(channel, DIO_MODE_INPUT);
    return (HAL_GPIO_ReadPin(kDioMap[channel].p_port, kDioMap[channel].pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void hal_do_write(uint16_t channel, uint8_t val)
{
    if (channel >= PLC_DIO_MAX_INDEX || kDioMap[channel].p_port == NULL) {
        return;
    }
    ensure_dio_mode(channel, DIO_MODE_OUTPUT);
    GPIO_PinState state = (val != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(kDioMap[channel].p_port, kDioMap[channel].pin, state);
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
    uint16_t dioNum = channel;
    if (channel < PLC_PWM_COUNT) {
        dioNum = kPwmIndexToDio[channel];
    }
    if (dioNum >= PLC_DIO_MAX_INDEX || kDioMap[dioNum].p_htim == NULL) {
        return 0U;
    }

    ensure_dio_mode(dioNum, DIO_MODE_PWM);

    TIM_HandleTypeDef *p_htim = kDioMap[dioNum].p_htim;
    if (freq_hz > 0) {
        uint32_t arr = (PLC_PWM_TIM_CLK_HZ / (uint32_t)freq_hz);
        if (arr < 2U)     { arr = 2U; }
        if (arr > 65536U) { arr = 65536U; }
        __HAL_TIM_SET_AUTORELOAD(p_htim, arr - 1U);
    }

    float clamped = dutyPct;
    if (clamped < 0.0f)        { clamped = 0.0f; }
    else if (clamped > 100.0f) { clamped = 100.0f; }

    uint32_t arr     = __HAL_TIM_GET_AUTORELOAD(p_htim);
    uint32_t compare = (uint32_t)((clamped / 100.0f) * (float)arr);
    __HAL_TIM_SET_COMPARE(p_htim, kDioMap[dioNum].timChannel, compare);
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
    memset(g_plc_hal_dioMode, 0, sizeof(g_plc_hal_dioMode));

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
        memset(g_plc_hal_dioMode, 0, sizeof(g_plc_hal_dioMode));
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
