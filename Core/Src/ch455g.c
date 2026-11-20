// Non-blocking TIM7-ISR 2-wire driver for WCH CH455G 7-Segment LED Display.

#include "ch455g.h"
#include "hw_tim7.h"
#include <string.h>

#define CH455G_QUEUE_CAP       16U
#define CH455G_HALF_CLK_US     6U

typedef struct {
    uint8_t cmd;
    uint8_t data;
} Ch455gFrame_t;

static const uint8_t kSegHexLut[16] = {
    0x3FU, // 0
    0x06U, // 1
    0x5BU, // 2
    0x4FU, // 3
    0x66U, // 4
    0x6DU, // 5
    0x7DU, // 6
    0x07U, // 7
    0x7FU, // 8
    0x6FU, // 9
    0x77U, // A
    0x7CU, // b
    0x39U, // C
    0x5EU, // d
    0x79U, // E
    0x71U  // F
};

static const uint8_t kDigCmds[4] = {
    CH455G_CMD_DIG0, CH455G_CMD_DIG1, CH455G_CMD_DIG2, CH455G_CMD_DIG3
};

static bool          g_ch455_initialized = false;
static GPIO_TypeDef *g_ch455_sclPort = NULL;
static uint16_t      g_ch455_sclPin  = 0U;
static GPIO_TypeDef *g_ch455_sdaPort = NULL;
static uint16_t      g_ch455_sdaPin  = 0U;

static uint8_t       g_ch455_lastSys = 0xFFU;
static uint8_t       g_ch455_lastSeg[4] = {0xFFU, 0xFFU, 0xFFU, 0xFFU};

static Ch455gFrame_t g_ch455_queue[CH455G_QUEUE_CAP];
static volatile uint8_t g_ch455_qHead = 0U;
static volatile uint8_t g_ch455_qTail = 0U;

// ISR state machine variables
static volatile uint8_t g_ch455_phase  = 0U;
static volatile uint8_t g_ch455_curCmd = 0U;
static volatile uint8_t g_ch455_curDat = 0U;
static volatile uint8_t g_ch455_bitIdx = 0U;

static inline void scl_high(void) { g_ch455_sclPort->BSRR = g_ch455_sclPin; }
static inline void scl_low(void)  { g_ch455_sclPort->BSRR = ((uint32_t)g_ch455_sclPin << 16U); }
static inline void sda_high(void) { g_ch455_sdaPort->BSRR = g_ch455_sdaPin; }
static inline void sda_low(void)  { g_ch455_sdaPort->BSRR = ((uint32_t)g_ch455_sdaPin << 16U); }

static void queue_push(uint8_t cmd, uint8_t data)
{
    uint8_t nextHead = (uint8_t)((g_ch455_qHead + 1U) % CH455G_QUEUE_CAP);
    if (nextHead != g_ch455_qTail) {
        g_ch455_queue[g_ch455_qHead].cmd  = cmd;
        g_ch455_queue[g_ch455_qHead].data = data;
        g_ch455_qHead = nextHead;
    }
}

// TIM7 ISR state machine: shifts out (cmd, data) with 9th-clock NACK tolerance.
static uint32_t ch455g_tim7_step(void)
{
    switch (g_ch455_phase) {
        case 0U: // Pop next frame & start condition setup (SDA=1, SCL=1)
            if (g_ch455_qTail == g_ch455_qHead) {
                scl_high();
                sda_high();
                return 0U;
            }
            g_ch455_curCmd = g_ch455_queue[g_ch455_qTail].cmd;
            g_ch455_curDat = g_ch455_queue[g_ch455_qTail].data;
            g_ch455_qTail  = (uint8_t)((g_ch455_qTail + 1U) % CH455G_QUEUE_CAP);
            sda_high();
            scl_high();
            g_ch455_phase = 1U;
            return CH455G_HALF_CLK_US;

        case 1U: // START: SDA=0 while SCL=1
            sda_low();
            g_ch455_phase  = 2U;
            g_ch455_bitIdx = 0U;
            return CH455G_HALF_CLK_US;

        case 2U: // Byte 1 (cmd) or Byte 2 (data): SCL=0, drive bit
        case 4U: {
            uint8_t byteVal = (g_ch455_phase == 2U) ? g_ch455_curCmd : g_ch455_curDat;
            scl_low();
            if (g_ch455_bitIdx < 8U) {
                uint8_t bit = (byteVal >> (7U - g_ch455_bitIdx)) & 1U;
                if (bit != 0U) { sda_high(); } else { sda_low(); }
            } else {
                // 9th clock cycle: release SDA (ignore ACK/NACK)
                sda_high();
            }
            g_ch455_phase++;
            return CH455G_HALF_CLK_US;
        }

        case 3U: // Rising edge of SCL for cmd/data bit or 9th clock
        case 5U:
            scl_high();
            g_ch455_bitIdx++;
            if (g_ch455_bitIdx < 9U) {
                g_ch455_phase--; // Next bit
            } else if (g_ch455_phase == 3U) {
                g_ch455_bitIdx = 0U;
                g_ch455_phase  = 4U; // Move to data byte
            } else {
                g_ch455_phase = 6U; // Move to STOP
            }
            return CH455G_HALF_CLK_US;

        case 6U: // STOP prep: SCL=0, SDA=0
            scl_low();
            sda_low();
            g_ch455_phase = 7U;
            return CH455G_HALF_CLK_US;

        case 7U: // STOP: SCL=1
            scl_high();
            g_ch455_phase = 8U;
            return CH455G_HALF_CLK_US;

        case 8U: // STOP: SDA=1
            sda_high();
            if (g_ch455_qTail != g_ch455_qHead) {
                g_ch455_phase = 0U;
                return CH455G_HALF_CLK_US;
            }
            g_ch455_phase = 0U;
            return 0U;

        default:
            g_ch455_phase = 0U;
            return 0U;
    }
}

void CH455G_ResetAll(void)
{
    g_ch455_initialized = false;
    g_ch455_sclPort     = NULL;
    g_ch455_sdaPort     = NULL;
    g_ch455_qHead       = 0U;
    g_ch455_qTail       = 0U;
    g_ch455_phase       = 0U;
    g_ch455_lastSys     = 0xFFU;
    memset(g_ch455_lastSeg, 0xFF, sizeof(g_ch455_lastSeg));
}

void CH455G_EnsureInit(GPIO_TypeDef *p_sclPort, uint16_t sclPin,
                       GPIO_TypeDef *p_sdaPort, uint16_t sdaPin)
{
    if (p_sclPort == NULL || p_sdaPort == NULL) {
        return;
    }
    if (g_ch455_initialized &&
        g_ch455_sclPort == p_sclPort && g_ch455_sclPin == sclPin &&
        g_ch455_sdaPort == p_sdaPort && g_ch455_sdaPin == sdaPin) {
        return;
    }

    g_ch455_sclPort = p_sclPort;
    g_ch455_sclPin  = sclPin;
    g_ch455_sdaPort = p_sdaPort;
    g_ch455_sdaPin  = sdaPin;

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = sclPin;
    HAL_GPIO_Init(p_sclPort, &gpio);
    gpio.Pin = sdaPin;
    HAL_GPIO_Init(p_sdaPort, &gpio);

    scl_high();
    sda_high();

    HW_TIM7_EnsureInit();

    g_ch455_lastSys = 0xFFU;
    memset(g_ch455_lastSeg, 0xFF, sizeof(g_ch455_lastSeg));
    g_ch455_initialized = true;
}

void CH455G_Update(bool enable, uint8_t brightness1to8, const uint8_t segBytes[4])
{
    if (!g_ch455_initialized || segBytes == NULL) {
        return;
    }

    uint8_t brt = brightness1to8;
    if (brt < 1U) { brt = 1U; }
    if (brt > 8U) { brt = 8U; }
    // CH455G INTENS[6:4]: 0 = 8/8 (max), 1..7 = 1/8..7/8
    uint8_t intens = (brt == 8U) ? 0U : brt;
    uint8_t sysVal = (uint8_t)((intens << 4U) | (enable ? 0x01U : 0x00U));

    if (sysVal != g_ch455_lastSys) {
        g_ch455_lastSys = sysVal;
        queue_push(CH455G_CMD_SYS, sysVal);
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        uint8_t seg = enable ? segBytes[i] : 0x00U;
        if (seg != g_ch455_lastSeg[i]) {
            g_ch455_lastSeg[i] = seg;
            queue_push(kDigCmds[i], seg);
        }
    }

    CH455G_PumpQueue();
}

void CH455G_PumpQueue(void)
{
    if (!g_ch455_initialized || g_ch455_qTail == g_ch455_qHead) {
        return;
    }
    if (!HW_TIM7_IsBusy()) {
        g_ch455_phase = 0U;
        HW_TIM7_ScheduleUs(CH455G_HALF_CLK_US, ch455g_tim7_step);
    }
}

uint8_t CH455G_EncodeDigit(int32_t code, bool dp)
{
    uint8_t seg = 0x00U;
    if (code >= 0 && code <= 15) {
        seg = kSegHexLut[code];
    } else if (code == -2) {
        seg = 0x40U; // Minus '-' (segment G)
    } else if (code >= 0x100) {
        seg = (uint8_t)(code & 0xFF); // Raw segment bitmask override
    }
    if (dp) {
        seg |= 0x80U;
    }
    return seg;
}
