// STM32F407 hardware abstraction glue for MaraVM, MaraProt, USB CDC, and BL24C256A EEPROM.

#include "plc_hal.h"
#include "bl24c256a.h"
#include "ch455g.h"
#include "cmsis_os.h"
#include "dac.h"
#include "ds18b20.h"
#include "hw_tim7.h"
#include "i2c.h"
#include "main.h"
#include "rng.h"
#include "rtc.h"
#include "tim.h"
#include "tmp75.h"
#include "usbd_cdc_if.h"
#include <math.h>
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
    DIO_MODE_PWM          = 3,
    DIO_MODE_PERIPH       = 4
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
static osMutexId_t       g_plc_hal_eepromMutex = NULL;
static bool              g_plc_hal_eepromReady = false;
static volatile bool     g_plc_hal_ready = false;
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
    return HW_DWT_Micros();
}

static int64_t rtc_days_from_civil(int32_t y, uint32_t m, uint32_t d)
{
    y -= (m <= 2U) ? 1 : 0;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    uint32_t yoe = (uint32_t)(y - (int32_t)(era * 400));
    uint32_t doy = (153U * (m + (m > 2U ? (uint32_t)-3 : 9U)) + 2U) / 5U + d - 1U;
    uint32_t doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

static void rtc_civil_from_days(int64_t z, int32_t *p_y, uint32_t *p_m, uint32_t *p_d)
{
    z += 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    uint32_t doe = (uint32_t)(z - era * 146097);
    uint32_t yoe = (doe - doe / 1460U + doe / 36524U - doe / 146096U) / 365U;
    int64_t y = (int64_t)yoe + era * 400;
    uint32_t doy = doe - (365U * yoe + yoe / 4U - yoe / 100U);
    uint32_t mp = (5U * doy + 2U) / 153U;
    *p_d = doy - (153U * mp + 2U) / 5U + 1U;
    *p_m = mp + (mp < 10U ? 3U : (uint32_t)-9);
    *p_y = (int32_t)(y + (*p_m <= 2U ? 1 : 0));
}

static int64_t hal_rtc_unix_sec(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    // STM32F4 shadow registers require reading Date immediately after Time
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    int32_t year = 2000 + (int32_t)sDate.Year;
    uint32_t mon = (sDate.Month >= 1U && sDate.Month <= 12U) ? sDate.Month : 1U;
    uint32_t day = (sDate.Date >= 1U && sDate.Date <= 31U) ? sDate.Date : 1U;
    int64_t days = rtc_days_from_civil(year, mon, day);
    return days * 86400 + (int64_t)sTime.Hours * 3600 + (int64_t)sTime.Minutes * 60 + (int64_t)sTime.Seconds;
}

static bool hal_rtc_set_unix_sec(int64_t unixSec)
{
    if (unixSec < 946684800LL) { // Before 2000-01-01 00:00:00 UTC (STM32 RTC base year 2000)
        return false;
    }
    int64_t days = unixSec / 86400;
    int64_t rem  = unixSec % 86400;
    int32_t yr   = 2000;
    uint32_t mon = 1U;
    uint32_t day = 1U;
    rtc_civil_from_days(days, &yr, &mon, &day);
    if (yr < 2000 || yr > 2099) {
        return false;
    }

    RTC_TimeTypeDef sTime = {0};
    sTime.Hours          = (uint8_t)(rem / 3600);
    sTime.Minutes        = (uint8_t)((rem % 3600) / 60);
    sTime.Seconds        = (uint8_t)(rem % 60);
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    // 1970-01-01 was Thursday (4); STM32 RTC_WEEKDAY_MONDAY=1 .. SUNDAY=7
    uint8_t dow0 = (uint8_t)((days + 3) % 7); // 0=Mon .. 6=Sun
    RTC_DateTypeDef sDate = {0};
    sDate.WeekDay = (uint8_t)(dow0 + 1U);
    sDate.Month   = (uint8_t)mon;
    sDate.Date    = (uint8_t)day;
    sDate.Year    = (uint8_t)(yr - 2000);

    HAL_PWR_EnableBkUpAccess();
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2U);
    return true;
}

static uint32_t hal_rng_next(void)
{
    uint32_t rnd = 0U;
    if (HAL_RNG_GenerateRandomNumber(&hrng, &rnd) != HAL_OK) {
        return HAL_GetTick();
    }
    return rnd;
}

static void eeprom_lock(void)
{
    if (g_plc_hal_eepromMutex != NULL && osKernelGetState() == osKernelRunning) {
        osMutexAcquire(g_plc_hal_eepromMutex, osWaitForever);
    }
    uint32_t startMs = HAL_GetTick();
    while (TMP75_IsDmaBusy() && (HAL_GetTick() - startMs) < 10U) {
        if (osKernelGetState() == osKernelRunning) {
            osDelay(1);
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        TMP75_OnI2cRxCplt(hi2c);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        TMP75_OnI2cError(hi2c);
    }
}

static void eeprom_unlock(void)
{
    if (g_plc_hal_eepromMutex != NULL && osKernelGetState() == osKernelRunning) {
        osMutexRelease(g_plc_hal_eepromMutex);
    }
}

static void recover_i2c1(void)
{
    HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();
}

static void hal_retentive_write(uint16_t paramId, uint64_t rawBits)
{
    if (!g_plc_hal_eepromReady) {
        return;
    }
    uint32_t addr = PLC_HAL_EEPROM_RET_ADDR + ((uint32_t)paramId * sizeof(uint64_t));
    if ((addr + sizeof(uint64_t)) <= BL24C256A_SIZE_BYTES) {
        eeprom_lock();
        BL24C256A_Update(&g_plc_hal_eeprom, (uint16_t)addr, (const uint8_t *)&rawBits, sizeof(rawBits));
        eeprom_unlock();
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
        eeprom_lock();
        BL24C256A_Read(&g_plc_hal_eeprom, (uint16_t)addr, (uint8_t *)&val, sizeof(val));
        eeprom_unlock();
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

    eeprom_lock();
    if (BL24C256A_Write(&g_plc_hal_eeprom, (uint16_t)addr, p_data, lenBytes) != BL24C256A_OK) {
        recover_i2c1();
        if (BL24C256A_Write(&g_plc_hal_eeprom, (uint16_t)addr, p_data, lenBytes) != BL24C256A_OK) {
            // Fallback to RAM-only execution if EEPROM fails, so uploads never brick live runs
            g_plc_hal_eepromReady = false;
        }
    }
    eeprom_unlock();
    return true;
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

    eeprom_lock();
    bool ok = (BL24C256A_Read(&g_plc_hal_eeprom, (uint16_t)addr, p_buf, lenBytes) == BL24C256A_OK);
    eeprom_unlock();
    return ok;
}

static void reset_shield_peripherals(void)
{
    HW_TIM7_DeInit();
    TMP75_ResetAll();
    DS18B20_ResetAll();
    CH455G_ResetAll();
}

static void format_ch455g_number(float val, uint8_t dec, uint8_t digitMask, uint8_t segOut[4])
{
    memset(segOut, 0, 4U);
    uint8_t activeSlots[4];
    uint8_t activeCount = 0U;
    for (uint8_t i = 0U; i < 4U; i++) {
        if ((digitMask & (1U << i)) != 0U) {
            activeSlots[activeCount++] = i;
        }
    }
    if (activeCount == 0U) {
        return;
    }

    if (dec >= activeCount) {
        dec = (uint8_t)(activeCount - 1U);
    }

    static const int32_t kPow10[4] = {1, 10, 100, 1000};
    float scaledF = val * (float)kPow10[dec];
    bool  neg     = (scaledF < -0.5f);
    int32_t mag   = (int32_t)llroundf(fabsf(scaledF));
    int32_t maxMag = kPow10[activeCount] - 1;
    if (neg && activeCount > 1U) {
        maxMag = kPow10[activeCount - 1U] - 1;
    } else if (neg && activeCount == 1U) {
        maxMag = 0;
    }

    if (mag > maxMag || isnan(val) || isinf(val)) {
        for (uint8_t i = 0U; i < activeCount; i++) {
            segOut[activeSlots[i]] = CH455G_EncodeDigit(-2, false); // "----"
        }
        return;
    }

    int32_t digits[4] = {-1, -1, -1, -1};
    uint8_t dpIdx = (dec > 0U) ? (uint8_t)(activeCount - 1U - dec) : 0xFFU;
    uint8_t pos = activeCount;
    int32_t tmp = mag;
    do {
        pos--;
        digits[pos] = tmp % 10;
        tmp /= 10;
    } while (pos > 0U && (tmp > 0 || (activeCount - pos) <= dec));

    if (neg && pos > 0U) {
        pos--;
        digits[pos] = -2;
    }

    for (uint8_t i = 0U; i < activeCount; i++) {
        bool hasDp = (dec > 0U) && (i == dpIdx);
        segOut[activeSlots[i]] = CH455G_EncodeDigit(digits[i], hasDp);
    }
}

static int32_t hal_comm_exec(uint8_t busType, uint16_t descId, int64_t txVal, int64_t *p_rxVal, int32_t *p_err)
{
    if (busType == VM_COMM_TMP75) {
        uint8_t addr7 = (uint8_t)(descId & 0x7FU);
        float tempC = 0.0f;
        bool ok = false;
        if (g_plc_hal_eepromMutex == NULL || osKernelGetState() != osKernelRunning ||
            osMutexAcquire(g_plc_hal_eepromMutex, 0U) == osOK) {
            TMP75_ReadCachedOrStartDma(&hi2c1, addr7, HAL_GetTick(), &tempC, &ok);
            if (g_plc_hal_eepromMutex != NULL && osKernelGetState() == osKernelRunning) {
                osMutexRelease(g_plc_hal_eepromMutex);
            }
        }
        uint32_t fBits = 0U;
        memcpy(&fBits, &tempC, sizeof(fBits));
        if (p_rxVal != NULL) { *p_rxVal = (int64_t)(uint64_t)fBits; }
        if (p_err != NULL)   { *p_err = ok ? 0 : 1; }
        return ok ? 0 : -1;
    }

    if (busType == VM_COMM_DS18B20) {
        uint8_t dioPin   = (uint8_t)(descId & 0xFFU);
        uint8_t busIndex = (uint8_t)((descId >> 8U) & 0xFFU);
        if (dioPin >= PLC_DIO_MAX_INDEX || kDioMap[dioPin].p_port == NULL) {
            if (p_err != NULL) { *p_err = 1; }
            return -1;
        }
        g_plc_hal_dioMode[dioPin] = DIO_MODE_PERIPH;

        float tempC = 0.0f;
        bool ok = false;
        DS18B20_PollAndRead(kDioMap[dioPin].p_port, kDioMap[dioPin].pin,
                            (uint64_t)txVal, busIndex, HAL_GetTick(), &tempC, &ok);
        uint32_t fBits = 0U;
        memcpy(&fBits, &tempC, sizeof(fBits));
        if (p_rxVal != NULL) { *p_rxVal = (int64_t)(uint64_t)fBits; }
        if (p_err != NULL)   { *p_err = ok ? 0 : 1; }
        return ok ? 0 : -1;
    }

    if (busType == VM_COMM_CH455G) {
        uint8_t sclDio = (uint8_t)(descId & 0xFFU);
        uint8_t sdaDio = (uint8_t)((descId >> 8U) & 0xFFU);
        if (sclDio >= PLC_DIO_MAX_INDEX || kDioMap[sclDio].p_port == NULL ||
            sdaDio >= PLC_DIO_MAX_INDEX || kDioMap[sdaDio].p_port == NULL) {
            if (p_err != NULL) { *p_err = 1; }
            return -1;
        }
        g_plc_hal_dioMode[sclDio] = DIO_MODE_PERIPH;
        g_plc_hal_dioMode[sdaDio] = DIO_MODE_PERIPH;
        CH455G_EnsureInit(kDioMap[sclDio].p_port, kDioMap[sclDio].pin,
                          kDioMap[sdaDio].p_port, kDioMap[sdaDio].pin);

        uint64_t packed = (uint64_t)txVal;
        uint8_t  brt       = (uint8_t)((packed >> 36U) & 0x0FU);
        uint8_t  digitMask = (uint8_t)((packed >> 40U) & 0x0FU);
        bool     enable    = ((packed >> 44U) & 0x01U) != 0U;
        uint8_t  mode      = (uint8_t)((packed >> 45U) & 0x01U);

        uint8_t segBytes[4] = {0U, 0U, 0U, 0U};
        if (enable) {
            if (mode == 0U) {
                uint32_t fBits = (uint32_t)(packed & 0xFFFFFFFFULL);
                float val = 0.0f;
                memcpy(&val, &fBits, sizeof(val));
                uint8_t dec = (uint8_t)((packed >> 32U) & 0x0FU);
                format_ch455g_number(val, dec, digitMask, segBytes);
            } else {
                for (uint8_t i = 0U; i < 4U; i++) {
                    segBytes[i] = ((digitMask & (1U << i)) != 0U)
                                ? (uint8_t)((packed >> (i * 8U)) & 0xFFU)
                                : 0x00U;
                }
            }
        }
        CH455G_Update(enable, brt, segBytes);
        if (p_err != NULL) { *p_err = 0; }
        return 0;
    }

    if (p_err != NULL) { *p_err = 1; }
    return -1;
}

static uint8_t hal_onewire_scan(uint8_t dioPin, uint8_t *p_outRoms8, uint8_t maxSensors)
{
    if (dioPin >= PLC_DIO_MAX_INDEX || kDioMap[dioPin].p_port == NULL || p_outRoms8 == NULL) {
        return 0U;
    }
    Ds18b20Sensor_t found[DS18B20_MAX_SENSORS];
    uint8_t limit = (maxSensors < DS18B20_MAX_SENSORS) ? maxSensors : DS18B20_MAX_SENSORS;
    uint8_t count = DS18B20_ScanBus(kDioMap[dioPin].p_port, kDioMap[dioPin].pin, found, limit);
    for (uint8_t i = 0U; i < count; i++) {
        memcpy(&p_outRoms8[(size_t)i * 8U], found[i].rom, 8U);
    }
    return count;
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
    HW_DWT_EnsureInit();
    reset_shield_peripherals();

    VmHalVtable_t vtable = {
        .di_read         = hal_di_read,
        .do_write        = hal_do_write,
        .ai_read         = hal_ai_read,
        .ao_write        = hal_ao_write,
        .pwm_set         = hal_pwm_set,
        .time_micros     = hal_time_micros,
        .rtc_unix_sec    = hal_rtc_unix_sec,
        .rng_next        = hal_rng_next,
        .retentive_write = hal_retentive_write,
        .retentive_read  = hal_retentive_read,
        .comm_exec       = hal_comm_exec
    };

    if (VM_Init(&g_plc_hal_vm, &vtable) != VM_OK) {
        return false;
    }

    g_plc_hal_eepromMutex = osMutexNew(NULL);
    g_plc_hal_eepromReady = (BL24C256A_Init(&g_plc_hal_eeprom, &hi2c1, 0U) == BL24C256A_OK);
    if (!g_plc_hal_eepromReady) {
        recover_i2c1();
        for (uint8_t addrPins = 0U; addrPins <= BL24C256A_ADDR_PINS_MAX; addrPins++) {
            if (BL24C256A_Init(&g_plc_hal_eeprom, &hi2c1, addrPins) == BL24C256A_OK) {
                g_plc_hal_eepromReady = true;
                break;
            }
        }
    }
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
        .rtc_get       = hal_rtc_unix_sec,
        .rtc_set       = hal_rtc_set_unix_sec,
        .onewire_scan  = hal_onewire_scan,
        .enter_dfu     = enter_rom_dfu
    };

    bool ok = PROT_Init(&g_plc_hal_prot, &g_plc_hal_vm, &transport);
    g_plc_hal_ready = ok;
    return ok;
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
        reset_shield_peripherals();
        PlcEepromHeader_t hdr = {
            .magic          = PLC_HAL_EEPROM_MAGIC,
            .imageSizeBytes = g_plc_hal_vm.imageSizeBytes,
            .imageCrc32     = g_plc_hal_vm.header.payloadCrc32,
            .reserved       = 0U
        };
        eeprom_lock();
        if (BL24C256A_UPDATE_STRUCT(&g_plc_hal_eeprom, PLC_HAL_EEPROM_HDR_ADDR, &hdr) != BL24C256A_OK) {
            recover_i2c1();
            if (BL24C256A_UPDATE_STRUCT(&g_plc_hal_eeprom, PLC_HAL_EEPROM_HDR_ADDR, &hdr) != BL24C256A_OK) {
                g_plc_hal_eepromReady = false;
            }
        }
        eeprom_unlock();
    }
}

void PLC_HAL_StepVmTick(uint32_t tick5msCount)
{
    if (!g_plc_hal_ready || g_plc_hal_prot.uploadActive || VM_GetState(&g_plc_hal_vm) != VM_STATE_RUNNING) {
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
