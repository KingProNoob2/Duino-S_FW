// Portable, hardware-agnostic MaraVM v1 bytecode interpreter implementation.

#include "mara_vm.h"
#include <math.h>
#include <string.h>

#define VM_INIT_ENTRY_BYTES   12U
#define VM_CRC32_POLY         0xEDB88320U
#define VM_CRC32_INIT         0xFFFFFFFFU
#define VM_LD_CONTACT_NO      0U
#define VM_LD_CONTACT_NC      1U
#define VM_LD_CONTACT_POS     2U
#define VM_LD_CONTACT_NEG     3U
#define VM_LD_COIL_NORMAL     0U
#define VM_LD_COIL_NEGATED    1U
#define VM_LD_COIL_SET        2U
#define VM_LD_COIL_RESET      3U
#define VM_LD_COIL_POS        4U
#define VM_LD_COIL_NEG        5U

static uint16_t read_u16_le(const uint8_t *p_buf)
{
    return (uint16_t)p_buf[0] | ((uint16_t)p_buf[1] << 8U);
}

static uint32_t read_u32_le(const uint8_t *p_buf)
{
    return (uint32_t)p_buf[0]
         | ((uint32_t)p_buf[1] << 8U)
         | ((uint32_t)p_buf[2] << 16U)
         | ((uint32_t)p_buf[3] << 24U);
}

static uint64_t read_u64_le(const uint8_t *p_buf)
{
    return (uint64_t)read_u32_le(p_buf) | ((uint64_t)read_u32_le(&p_buf[4]) << 32U);
}

static void parse_header(const uint8_t *p_buf, VmImageHeader_t *p_hdr)
{
    p_hdr->magic           = read_u32_le(&p_buf[0]);
    p_hdr->formatVersion   = read_u16_le(&p_buf[4]);
    p_hdr->targetModelId   = read_u16_le(&p_buf[6]);
    p_hdr->imageSizeBytes  = read_u32_le(&p_buf[8]);
    p_hdr->payloadCrc32    = read_u32_le(&p_buf[12]);
    p_hdr->projectHash     = read_u32_le(&p_buf[16]);
    p_hdr->numBoolSlots    = read_u16_le(&p_buf[20]);
    p_hdr->numIntSlots     = read_u16_le(&p_buf[22]);
    p_hdr->numLongSlots    = read_u16_le(&p_buf[24]);
    p_hdr->numFloatSlots   = read_u16_le(&p_buf[26]);
    p_hdr->numStateSlots   = read_u16_le(&p_buf[28]);
    p_hdr->numArrays       = read_u16_le(&p_buf[30]);
    p_hdr->estopConfig     = read_u32_le(&p_buf[32]);
    p_hdr->initTableOffset = read_u32_le(&p_buf[36]);
    p_hdr->initTableCount  = read_u32_le(&p_buf[40]);
    for (uint32_t i = 0U; i < VM_THREAD_COUNT; i++) {
        p_hdr->threadOffset[i] = read_u32_le(&p_buf[44U + (i * 4U)]);
    }
}

static bool slot_in_bounds(uint16_t slotId)
{
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:  return idx < VM_MAX_BOOL_SLOTS;
        case VM_TYPE_INT:   return idx < VM_MAX_INT_SLOTS;
        case VM_TYPE_LONG:  return idx < VM_MAX_LONG_SLOTS;
        case VM_TYPE_FLOAT: return idx < VM_MAX_FLOAT_SLOTS;
        default:            return false;
    }
}

static uint8_t read_bool(const VmInstance_t *p_vm, uint16_t slotId)
{
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:  return (idx < VM_MAX_BOOL_SLOTS) ? (p_vm->boolRam[idx] != 0U ? 1U : 0U) : 0U;
        case VM_TYPE_INT:   return (idx < VM_MAX_INT_SLOTS) ? (p_vm->intRam[idx] != 0 ? 1U : 0U) : 0U;
        case VM_TYPE_LONG:  return (idx < VM_MAX_LONG_SLOTS) ? (p_vm->longRam[idx] != 0 ? 1U : 0U) : 0U;
        case VM_TYPE_FLOAT: return (idx < VM_MAX_FLOAT_SLOTS) ? (p_vm->floatRam[idx] != 0.0f ? 1U : 0U) : 0U;
        default:            return 0U;
    }
}

static int32_t read_int(const VmInstance_t *p_vm, uint16_t slotId)
{
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:  return (idx < VM_MAX_BOOL_SLOTS) ? (int32_t)(p_vm->boolRam[idx] != 0U ? 1 : 0) : 0;
        case VM_TYPE_INT:   return (idx < VM_MAX_INT_SLOTS) ? p_vm->intRam[idx] : 0;
        case VM_TYPE_LONG:  return (idx < VM_MAX_LONG_SLOTS) ? (int32_t)p_vm->longRam[idx] : 0;
        case VM_TYPE_FLOAT: return (idx < VM_MAX_FLOAT_SLOTS) ? (int32_t)p_vm->floatRam[idx] : 0;
        default:            return 0;
    }
}

static int64_t read_long(const VmInstance_t *p_vm, uint16_t slotId)
{
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:  return (idx < VM_MAX_BOOL_SLOTS) ? (int64_t)(p_vm->boolRam[idx] != 0U ? 1 : 0) : 0;
        case VM_TYPE_INT:   return (idx < VM_MAX_INT_SLOTS) ? (int64_t)p_vm->intRam[idx] : 0;
        case VM_TYPE_LONG:  return (idx < VM_MAX_LONG_SLOTS) ? p_vm->longRam[idx] : 0;
        case VM_TYPE_FLOAT: return (idx < VM_MAX_FLOAT_SLOTS) ? (int64_t)p_vm->floatRam[idx] : 0;
        default:            return 0;
    }
}

static float read_float(const VmInstance_t *p_vm, uint16_t slotId)
{
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:  return (idx < VM_MAX_BOOL_SLOTS) ? (p_vm->boolRam[idx] != 0U ? 1.0f : 0.0f) : 0.0f;
        case VM_TYPE_INT:   return (idx < VM_MAX_INT_SLOTS) ? (float)p_vm->intRam[idx] : 0.0f;
        case VM_TYPE_LONG:  return (idx < VM_MAX_LONG_SLOTS) ? (float)p_vm->longRam[idx] : 0.0f;
        case VM_TYPE_FLOAT: return (idx < VM_MAX_FLOAT_SLOTS) ? p_vm->floatRam[idx] : 0.0f;
        default:            return 0.0f;
    }
}

static void write_bool(VmInstance_t *p_vm, uint16_t slotId, uint8_t val)
{
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    uint8_t  bVal = (val != 0U) ? 1U : 0U;
    switch (type) {
        case VM_TYPE_BOOL:  if (idx < VM_MAX_BOOL_SLOTS) { p_vm->boolRam[idx] = bVal; } break;
        case VM_TYPE_INT:   if (idx < VM_MAX_INT_SLOTS) { p_vm->intRam[idx] = (int32_t)bVal; } break;
        case VM_TYPE_LONG:  if (idx < VM_MAX_LONG_SLOTS) { p_vm->longRam[idx] = (int64_t)bVal; } break;
        case VM_TYPE_FLOAT: if (idx < VM_MAX_FLOAT_SLOTS) { p_vm->floatRam[idx] = (float)bVal; } break;
        default:            break;
    }
}

static void write_int(VmInstance_t *p_vm, uint16_t slotId, int32_t val)
{
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:  if (idx < VM_MAX_BOOL_SLOTS) { p_vm->boolRam[idx] = (val != 0) ? 1U : 0U; } break;
        case VM_TYPE_INT:   if (idx < VM_MAX_INT_SLOTS) { p_vm->intRam[idx] = val; } break;
        case VM_TYPE_LONG:  if (idx < VM_MAX_LONG_SLOTS) { p_vm->longRam[idx] = (int64_t)val; } break;
        case VM_TYPE_FLOAT: if (idx < VM_MAX_FLOAT_SLOTS) { p_vm->floatRam[idx] = (float)val; } break;
        default:            break;
    }
}

static void write_long(VmInstance_t *p_vm, uint16_t slotId, int64_t val)
{
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:  if (idx < VM_MAX_BOOL_SLOTS) { p_vm->boolRam[idx] = (val != 0) ? 1U : 0U; } break;
        case VM_TYPE_INT:   if (idx < VM_MAX_INT_SLOTS) { p_vm->intRam[idx] = (int32_t)val; } break;
        case VM_TYPE_LONG:  if (idx < VM_MAX_LONG_SLOTS) { p_vm->longRam[idx] = val; } break;
        case VM_TYPE_FLOAT: if (idx < VM_MAX_FLOAT_SLOTS) { p_vm->floatRam[idx] = (float)val; } break;
        default:            break;
    }
}

static void write_float(VmInstance_t *p_vm, uint16_t slotId, float val)
{
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:  if (idx < VM_MAX_BOOL_SLOTS) { p_vm->boolRam[idx] = (val != 0.0f) ? 1U : 0U; } break;
        case VM_TYPE_INT:   if (idx < VM_MAX_INT_SLOTS) { p_vm->intRam[idx] = (int32_t)val; } break;
        case VM_TYPE_LONG:  if (idx < VM_MAX_LONG_SLOTS) { p_vm->longRam[idx] = (int64_t)val; } break;
        case VM_TYPE_FLOAT: if (idx < VM_MAX_FLOAT_SLOTS) { p_vm->floatRam[idx] = val; } break;
        default:            break;
    }
}

static void apply_init_table(VmInstance_t *p_vm)
{
    uint32_t offset = p_vm->header.initTableOffset;
    uint32_t count  = p_vm->header.initTableCount;
    for (uint32_t i = 0U; i < count; i++) {
        uint32_t entryOff = offset + (i * VM_INIT_ENTRY_BYTES);
        if ((entryOff + VM_INIT_ENTRY_BYTES) > p_vm->imageSizeBytes) {
            break;
        }
        uint16_t slotId = read_u16_le(&p_vm->imageBuf[entryOff]);
        uint64_t rawVal = read_u64_le(&p_vm->imageBuf[entryOff + 4U]);
        uint8_t  rawBytes[VM_MAX_SLOT_BYTES];
        memcpy(rawBytes, &rawVal, sizeof(rawBytes));
        VM_WriteSlotRaw(p_vm, slotId, rawBytes);
    }
}

static void step_pid(VmInstance_t *p_vm, uint16_t dstSlot, uint16_t pvSlot, uint16_t spSlot,
                     uint16_t kpSlot, uint16_t kiSlot, uint16_t kdSlot, uint16_t rstSlot,
                     uint16_t stateIdx, uint32_t dt_us)
{
    if (stateIdx >= VM_MAX_STATE_SLOTS) {
        return;
    }
    VmStateSlot_t *p_st = &p_vm->stateRam[stateIdx];
    if (read_bool(p_vm, rstSlot) != 0U) {
        p_st->accFloat = 0.0f;
        p_st->auxFloat = 0.0f;
        write_float(p_vm, dstSlot, 0.0f);
        return;
    }

    float dt_s  = (dt_us > 0U) ? ((float)dt_us / 1000000.0f) : 0.01f;
    float error = read_float(p_vm, spSlot) - read_float(p_vm, pvSlot);
    float kp    = read_float(p_vm, kpSlot);
    float ki    = read_float(p_vm, kiSlot);
    float kd    = read_float(p_vm, kdSlot);

    p_st->accFloat += error * dt_s;
    float deriv = (error - p_st->auxFloat) / dt_s;
    p_st->auxFloat = error;

    float outVal = (kp * error) + (ki * p_st->accFloat) + (kd * deriv);
    write_float(p_vm, dstSlot, outVal);
}

uint32_t VM_CalcCrc32(const uint8_t *p_data, uint32_t lenBytes)
{
    if (p_data == NULL) {
        return 0U;
    }
    uint32_t crc = VM_CRC32_INIT;
    for (uint32_t i = 0U; i < lenBytes; i++) {
        crc ^= (uint32_t)p_data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ VM_CRC32_POLY;
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc ^ VM_CRC32_INIT;
}

VmStatus_t VM_Init(VmInstance_t *p_vm, const VmHalVtable_t *p_hal)
{
    if (p_vm == NULL || p_hal == NULL) {
        return VM_ERR_PARAM;
    }
    memset(p_vm, 0, sizeof(*p_vm));
    p_vm->hal      = *p_hal;
    p_vm->runState = VM_STATE_STOPPED;
    return VM_OK;
}

VmStatus_t VM_LoadImage(VmInstance_t *p_vm, const uint8_t *p_image, uint32_t lenBytes)
{
    if (p_vm == NULL || p_image == NULL) {
        return VM_ERR_PARAM;
    }
    if (lenBytes < VM_HEADER_SIZE_BYTES || lenBytes > VM_MAX_IMAGE_BYTES) {
        return VM_ERR_TOO_LARGE;
    }

    VmImageHeader_t hdr;
    parse_header(p_image, &hdr);
    if (hdr.magic != VM_MAGIC_MFLW) {
        return VM_ERR_BAD_MAGIC;
    }
    if (hdr.formatVersion != VM_FORMAT_VERSION) {
        return VM_ERR_BAD_VERSION;
    }
    if (hdr.imageSizeBytes != lenBytes) {
        return VM_ERR_BAD_CRC;
    }

    uint32_t calcCrc = VM_CalcCrc32(&p_image[VM_HEADER_SIZE_BYTES], lenBytes - VM_HEADER_SIZE_BYTES);
    if (calcCrc != hdr.payloadCrc32) {
        return VM_ERR_BAD_CRC;
    }

    p_vm->runState = VM_STATE_STOPPED;
    memcpy(p_vm->imageBuf, p_image, lenBytes);
    p_vm->imageSizeBytes = lenBytes;
    p_vm->header         = hdr;
    p_vm->hasValidImage  = true;

    return VM_Reset(p_vm);
}

VmStatus_t VM_Reset(VmInstance_t *p_vm)
{
    if (p_vm == NULL) {
        return VM_ERR_PARAM;
    }
    memset(p_vm->boolRam, 0, sizeof(p_vm->boolRam));
    memset(p_vm->intRam, 0, sizeof(p_vm->intRam));
    memset(p_vm->longRam, 0, sizeof(p_vm->longRam));
    memset(p_vm->floatRam, 0, sizeof(p_vm->floatRam));
    memset(p_vm->stateRam, 0, sizeof(p_vm->stateRam));
    p_vm->lastFaultCode = VM_OK;
    p_vm->lastFaultPc   = 0U;
    p_vm->scanCount     = 0U;

    if (p_vm->hasValidImage) {
        apply_init_table(p_vm);
    }
    return VM_OK;
}

VmStatus_t VM_SetState(VmInstance_t *p_vm, VmRunState_t newState)
{
    if (p_vm == NULL) {
        return VM_ERR_PARAM;
    }
    if (newState == VM_STATE_RUNNING && !p_vm->hasValidImage) {
        return VM_ERR_FAULT_BOUNDS;
    }
    p_vm->runState = newState;
    return VM_OK;
}

VmRunState_t VM_GetState(const VmInstance_t *p_vm)
{
    return (p_vm != NULL) ? p_vm->runState : VM_STATE_FAULT;
}

uint8_t VM_GetSlotSize(uint16_t slotId)
{
    uint8_t type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:  return 1U;
        case VM_TYPE_INT:   return 4U;
        case VM_TYPE_LONG:  return 8U;
        case VM_TYPE_FLOAT: return 4U;
        default:            return 0U;
    }
}

uint8_t VM_ReadSlotRaw(const VmInstance_t *p_vm, uint16_t slotId, uint8_t *p_outBuf)
{
    if (p_vm == NULL || p_outBuf == NULL || !slot_in_bounds(slotId)) {
        return 0U;
    }
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:
            p_outBuf[0] = p_vm->boolRam[idx];
            return 1U;
        case VM_TYPE_INT:
            memcpy(p_outBuf, &p_vm->intRam[idx], sizeof(int32_t));
            return 4U;
        case VM_TYPE_LONG:
            memcpy(p_outBuf, &p_vm->longRam[idx], sizeof(int64_t));
            return 8U;
        case VM_TYPE_FLOAT:
            memcpy(p_outBuf, &p_vm->floatRam[idx], sizeof(float));
            return 4U;
        default:
            return 0U;
    }
}

uint8_t VM_WriteSlotRaw(VmInstance_t *p_vm, uint16_t slotId, const uint8_t *p_inBuf)
{
    if (p_vm == NULL || p_inBuf == NULL || !slot_in_bounds(slotId)) {
        return 0U;
    }
    uint16_t idx  = slotId & VM_SLOT_INDEX_MASK;
    uint8_t  type = (uint8_t)(slotId >> VM_SLOT_TYPE_SHIFT);
    switch (type) {
        case VM_TYPE_BOOL:
            p_vm->boolRam[idx] = (p_inBuf[0] != 0U) ? 1U : 0U;
            return 1U;
        case VM_TYPE_INT:
            memcpy(&p_vm->intRam[idx], p_inBuf, sizeof(int32_t));
            return 4U;
        case VM_TYPE_LONG:
            memcpy(&p_vm->longRam[idx], p_inBuf, sizeof(int64_t));
            return 8U;
        case VM_TYPE_FLOAT:
            memcpy(&p_vm->floatRam[idx], p_inBuf, sizeof(float));
            return 4U;
        default:
            return 0U;
    }
}

VmStatus_t VM_StepThread(VmInstance_t *p_vm, uint8_t threadId, uint32_t dt_us)
{
    if (p_vm == NULL || threadId >= VM_THREAD_COUNT) {
        return VM_ERR_PARAM;
    }
    if (!p_vm->hasValidImage || p_vm->runState != VM_STATE_RUNNING) {
        return VM_OK;
    }

    uint32_t pc = p_vm->header.threadOffset[threadId];
    if (pc == 0U || pc >= p_vm->imageSizeBytes) {
        return VM_OK;
    }

    uint32_t start_us = (p_vm->hal.time_micros != NULL) ? p_vm->hal.time_micros() : 0U;
    uint32_t fuel     = VM_MAX_LOOP_FUEL;
    uint32_t dt_ms    = dt_us / 1000U;
    if (dt_ms == 0U && dt_us > 0U) {
        dt_ms = 1U;
    }

    while ((pc + 8U) <= p_vm->imageSizeBytes) {
        uint8_t  opcode = p_vm->imageBuf[pc];
        uint8_t  flags  = p_vm->imageBuf[pc + 1U];
        uint16_t enSlot = read_u16_le(&p_vm->imageBuf[pc + 2U]);
        uint16_t dst    = read_u16_le(&p_vm->imageBuf[pc + 4U]);
        uint16_t srcA   = read_u16_le(&p_vm->imageBuf[pc + 6U]);

        if (opcode == VM_OP_HALT) {
            break;
        }
        if (opcode == VM_OP_NOP) {
            pc += 8U;
            continue;
        }

        // Fixed word counts per instruction format
        uint32_t nextPc = pc + 8U;
        bool     enabled = (enSlot == VM_SLOT_ALWAYS_ENABLED) || (read_bool(p_vm, enSlot) != 0U);

        switch (opcode) {
            case VM_OP_JMP: {
                int16_t relBytes = (int16_t)srcA;
                if (relBytes <= 0) {
                    if (fuel == 0U) {
                        p_vm->lastFaultCode = VM_ERR_FAULT_FUEL;
                        p_vm->lastFaultPc   = (uint16_t)pc;
                        p_vm->runState      = VM_STATE_FAULT;
                        return VM_ERR_FAULT_FUEL;
                    }
                    fuel--;
                }
                nextPc = (uint32_t)((int32_t)pc + (int32_t)relBytes);
                break;
            }
            case VM_OP_JMP_FALSE: {
                int16_t relBytes = (int16_t)srcA;
                if (read_bool(p_vm, dst) == 0U) {
                    if (relBytes <= 0) {
                        if (fuel == 0U) {
                            p_vm->lastFaultCode = VM_ERR_FAULT_FUEL;
                            p_vm->lastFaultPc   = (uint16_t)pc;
                            p_vm->runState      = VM_STATE_FAULT;
                            return VM_ERR_FAULT_FUEL;
                        }
                        fuel--;
                    }
                    nextPc = (uint32_t)((int32_t)pc + (int32_t)relBytes);
                }
                break;
            }
            case VM_OP_MOV:
            case VM_OP_CAST: {
                if (enabled) {
                    uint8_t dstType = (uint8_t)(dst >> VM_SLOT_TYPE_SHIFT);
                    if (dstType == VM_TYPE_BOOL)       { write_bool(p_vm, dst, read_bool(p_vm, srcA)); }
                    else if (dstType == VM_TYPE_INT)   { write_int(p_vm, dst, read_int(p_vm, srcA)); }
                    else if (dstType == VM_TYPE_LONG)  { write_long(p_vm, dst, read_long(p_vm, srcA)); }
                    else                               { write_float(p_vm, dst, read_float(p_vm, srcA)); }
                }
                break;
            }
            case VM_OP_NOT: {
                if (enabled) {
                    write_bool(p_vm, dst, (read_bool(p_vm, srcA) == 0U) ? 1U : 0U);
                }
                break;
            }
            case VM_OP_AND:
            case VM_OP_NAND:
            case VM_OP_OR:
            case VM_OP_NOR:
            case VM_OP_XOR:
            case VM_OP_XNOR: {
                // Word 2 holds srcB + optional extra slots based on flags (input count)
                uint8_t argc = (flags >= 2U) ? flags : 2U;
                uint32_t extraWords = (uint32_t)((argc - 1U + 1U) / 2U);
                nextPc = pc + 8U + (extraWords * 4U);
                if (nextPc > p_vm->imageSizeBytes) { break; }
                if (enabled) {
                    uint8_t acc = read_bool(p_vm, srcA);
                    for (uint8_t k = 1U; k < argc; k++) {
                        uint16_t s = read_u16_le(&p_vm->imageBuf[pc + 8U + ((uint32_t)(k - 1U) * 2U)]);
                        uint8_t  v = read_bool(p_vm, s);
                        if (opcode == VM_OP_AND || opcode == VM_OP_NAND)      { acc &= v; }
                        else if (opcode == VM_OP_OR || opcode == VM_OP_NOR)   { acc |= v; }
                        else                                                  { acc ^= v; }
                    }
                    if (opcode == VM_OP_NAND || opcode == VM_OP_NOR || opcode == VM_OP_XNOR) {
                        acc = (acc == 0U) ? 1U : 0U;
                    }
                    write_bool(p_vm, dst, acc);
                }
                break;
            }
            case VM_OP_RS:
            case VM_OP_SR: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t rSlot    = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                uint16_t nqSlot   = read_u16_le(&p_vm->imageBuf[pc + 10U]);
                if (enabled) {
                    uint8_t sVal = read_bool(p_vm, srcA);
                    uint8_t rVal = read_bool(p_vm, rSlot);
                    uint8_t qVal = read_bool(p_vm, dst);
                    if (opcode == VM_OP_RS) {
                        if (rVal != 0U)      { qVal = 0U; }
                        else if (sVal != 0U) { qVal = 1U; }
                    } else {
                        if (sVal != 0U)      { qVal = 1U; }
                        else if (rVal != 0U) { qVal = 0U; }
                    }
                    write_bool(p_vm, dst, qVal);
                    if (nqSlot != VM_SLOT_ALWAYS_ENABLED) {
                        write_bool(p_vm, nqSlot, (qVal == 0U) ? 1U : 0U);
                    }
                }
                break;
            }
            case VM_OP_LD_CONTACT: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t stateIdx = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                uint8_t  opVal    = read_bool(p_vm, srcA);
                uint8_t  pass     = 0U;
                if (flags == VM_LD_CONTACT_NO) {
                    pass = opVal;
                } else if (flags == VM_LD_CONTACT_NC) {
                    pass = (opVal == 0U) ? 1U : 0U;
                } else if (stateIdx < VM_MAX_STATE_SLOTS) {
                    uint8_t prev = p_vm->stateRam[stateIdx].prevBool;
                    p_vm->stateRam[stateIdx].prevBool = opVal;
                    pass = (flags == VM_LD_CONTACT_POS) ? ((opVal != 0U && prev == 0U) ? 1U : 0U)
                                                        : ((opVal == 0U && prev != 0U) ? 1U : 0U);
                }
                write_bool(p_vm, dst, (enabled && pass != 0U) ? 1U : 0U);
                break;
            }
            case VM_OP_LD_COIL: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t stateIdx = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                uint8_t  pwr      = enabled ? 1U : 0U;
                if (flags == VM_LD_COIL_NORMAL) {
                    write_bool(p_vm, dst, pwr);
                } else if (flags == VM_LD_COIL_NEGATED) {
                    write_bool(p_vm, dst, (pwr == 0U) ? 1U : 0U);
                } else if (flags == VM_LD_COIL_SET && pwr != 0U) {
                    write_bool(p_vm, dst, 1U);
                } else if (flags == VM_LD_COIL_RESET && pwr != 0U) {
                    write_bool(p_vm, dst, 0U);
                } else if ((flags == VM_LD_COIL_POS || flags == VM_LD_COIL_NEG) && stateIdx < VM_MAX_STATE_SLOTS) {
                    uint8_t prev = p_vm->stateRam[stateIdx].prevBool;
                    p_vm->stateRam[stateIdx].prevBool = pwr;
                    uint8_t pulse = (flags == VM_LD_COIL_POS) ? ((pwr != 0U && prev == 0U) ? 1U : 0U)
                                                               : ((pwr == 0U && prev != 0U) ? 1U : 0U);
                    write_bool(p_vm, dst, pulse);
                }
                break;
            }
            case VM_OP_ADD:
            case VM_OP_SUB:
            case VM_OP_MUL:
            case VM_OP_MIN:
            case VM_OP_MAX: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t srcB = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                if (enabled) {
                    uint8_t dstType = (uint8_t)(dst >> VM_SLOT_TYPE_SHIFT);
                    if (dstType == VM_TYPE_FLOAT) {
                        float a = read_float(p_vm, srcA);
                        float b = read_float(p_vm, srcB);
                        float r = 0.0f;
                        if (opcode == VM_OP_ADD)      { r = a + b; }
                        else if (opcode == VM_OP_SUB) { r = a - b; }
                        else if (opcode == VM_OP_MUL) { r = a * b; }
                        else if (opcode == VM_OP_MIN) { r = (a < b) ? a : b; }
                        else                          { r = (a > b) ? a : b; }
                        write_float(p_vm, dst, r);
                    } else {
                        int32_t a = read_int(p_vm, srcA);
                        int32_t b = read_int(p_vm, srcB);
                        int32_t r = 0;
                        if (opcode == VM_OP_ADD)      { r = a + b; }
                        else if (opcode == VM_OP_SUB) { r = a - b; }
                        else if (opcode == VM_OP_MUL) { r = a * b; }
                        else if (opcode == VM_OP_MIN) { r = (a < b) ? a : b; }
                        else                          { r = (a > b) ? a : b; }
                        write_int(p_vm, dst, r);
                    }
                }
                break;
            }
            case VM_OP_DIV:
            case VM_OP_MOD: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t srcB    = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                uint16_t errSlot = read_u16_le(&p_vm->imageBuf[pc + 10U]);
                if (enabled) {
                    int32_t b = read_int(p_vm, srcB);
                    if (b == 0) {
                        write_int(p_vm, dst, 0);
                        if (errSlot != VM_SLOT_ALWAYS_ENABLED) { write_bool(p_vm, errSlot, 1U); }
                    } else {
                        int32_t a = read_int(p_vm, srcA);
                        write_int(p_vm, dst, (opcode == VM_OP_DIV) ? (a / b) : (a % b));
                        if (errSlot != VM_SLOT_ALWAYS_ENABLED) { write_bool(p_vm, errSlot, 0U); }
                    }
                }
                break;
            }
            case VM_OP_ABS:
            case VM_OP_NEG:
            case VM_OP_SQRT:
            case VM_OP_SIN:
            case VM_OP_COS: {
                if (enabled) {
                    float a = read_float(p_vm, srcA);
                    float r = a;
                    if (opcode == VM_OP_ABS)       { r = fabsf(a); }
                    else if (opcode == VM_OP_NEG)  { r = -a; }
                    else if (opcode == VM_OP_SQRT) { r = (a >= 0.0f) ? sqrtf(a) : 0.0f; }
                    else if (opcode == VM_OP_SIN)  { r = sinf(a); }
                    else                           { r = cosf(a); }
                    write_float(p_vm, dst, r);
                }
                break;
            }
            case VM_OP_CLAMP: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t minSlot = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                uint16_t maxSlot = read_u16_le(&p_vm->imageBuf[pc + 10U]);
                if (enabled) {
                    float v  = read_float(p_vm, srcA);
                    float lo = read_float(p_vm, minSlot);
                    float hi = read_float(p_vm, maxSlot);
                    if (v < lo)      { v = lo; }
                    else if (v > hi) { v = hi; }
                    write_float(p_vm, dst, v);
                }
                break;
            }
            case VM_OP_PID: {
                nextPc = pc + 20U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                if (enabled) {
                    uint16_t spSlot   = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                    uint16_t kpSlot   = read_u16_le(&p_vm->imageBuf[pc + 10U]);
                    uint16_t kiSlot   = read_u16_le(&p_vm->imageBuf[pc + 12U]);
                    uint16_t kdSlot   = read_u16_le(&p_vm->imageBuf[pc + 14U]);
                    uint16_t rstSlot  = read_u16_le(&p_vm->imageBuf[pc + 16U]);
                    uint16_t stateIdx = read_u16_le(&p_vm->imageBuf[pc + 18U]);
                    step_pid(p_vm, dst, srcA, spSlot, kpSlot, kiSlot, kdSlot, rstSlot, stateIdx, dt_us);
                }
                break;
            }
            case VM_OP_CMP_EQ:
            case VM_OP_CMP_NE:
            case VM_OP_CMP_GT:
            case VM_OP_CMP_GE:
            case VM_OP_CMP_LT:
            case VM_OP_CMP_LE: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t srcB = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                if (enabled) {
                    float a = read_float(p_vm, srcA);
                    float b = read_float(p_vm, srcB);
                    bool  res = false;
                    if (opcode == VM_OP_CMP_EQ)      { res = (a == b); }
                    else if (opcode == VM_OP_CMP_NE) { res = (a != b); }
                    else if (opcode == VM_OP_CMP_GT) { res = (a > b); }
                    else if (opcode == VM_OP_CMP_GE) { res = (a >= b); }
                    else if (opcode == VM_OP_CMP_LT) { res = (a < b); }
                    else                             { res = (a <= b); }
                    write_bool(p_vm, dst, res ? 1U : 0U);
                }
                break;
            }
            case VM_OP_SEL: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t x0Slot = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                uint16_t x1Slot = read_u16_le(&p_vm->imageBuf[pc + 10U]);
                if (enabled) {
                    uint16_t chosen = (read_bool(p_vm, srcA) != 0U) ? x1Slot : x0Slot;
                    write_float(p_vm, dst, read_float(p_vm, chosen));
                }
                break;
            }
            case VM_OP_TON: {
                nextPc = pc + 16U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t ptSlot   = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                uint16_t etSlot   = read_u16_le(&p_vm->imageBuf[pc + 10U]);
                uint16_t stateIdx = read_u16_le(&p_vm->imageBuf[pc + 12U]);
                if (stateIdx < VM_MAX_STATE_SLOTS) {
                    VmStateSlot_t *p_st = &p_vm->stateRam[stateIdx];
                    if (enabled && read_bool(p_vm, srcA) != 0U) {
                        int32_t pt_ms = read_int(p_vm, ptSlot);
                        if (pt_ms < 0) { pt_ms = 0; }
                        p_st->timer_ms += dt_ms;
                        if (p_st->timer_ms >= (uint32_t)pt_ms) {
                            p_st->timer_ms = (uint32_t)pt_ms;
                            write_bool(p_vm, dst, 1U);
                        } else {
                            write_bool(p_vm, dst, 0U);
                        }
                    } else {
                        p_st->timer_ms = 0U;
                        write_bool(p_vm, dst, 0U);
                    }
                    if (etSlot != VM_SLOT_ALWAYS_ENABLED) {
                        write_int(p_vm, etSlot, (int32_t)p_st->timer_ms);
                    }
                }
                break;
            }
            case VM_OP_ONE_SHOT: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t stateIdx = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                if (stateIdx < VM_MAX_STATE_SLOTS) {
                    uint8_t cur  = enabled ? read_bool(p_vm, srcA) : 0U;
                    uint8_t prev = p_vm->stateRam[stateIdx].prevBool;
                    p_vm->stateRam[stateIdx].prevBool = cur;
                    write_bool(p_vm, dst, (cur != 0U && prev == 0U) ? 1U : 0U);
                }
                break;
            }
            case VM_OP_CTUD: {
                nextPc = pc + 16U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t cdSlot   = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                uint16_t rSlot    = read_u16_le(&p_vm->imageBuf[pc + 10U]);
                uint16_t pvSlot   = read_u16_le(&p_vm->imageBuf[pc + 12U]);
                uint16_t cvSlot   = read_u16_le(&p_vm->imageBuf[pc + 14U]);
                // Counter state uses cvSlot directly and edge flags from prevBool/prevAuxBool if stateIdx encoded
                if (enabled) {
                    int32_t cv = read_int(p_vm, cvSlot);
                    int32_t pv = read_int(p_vm, pvSlot);
                    if (read_bool(p_vm, rSlot) != 0U) {
                        cv = 0;
                    } else {
                        if (read_bool(p_vm, srcA) != 0U)   { cv++; }
                        if (read_bool(p_vm, cdSlot) != 0U) { cv--; }
                    }
                    write_int(p_vm, cvSlot, cv);
                    write_bool(p_vm, dst, (cv >= pv) ? 1U : 0U);
                }
                break;
            }
            case VM_OP_IO_READ: {
                if (enabled) {
                    if (flags == VM_IO_DI && p_vm->hal.di_read != NULL) {
                        write_bool(p_vm, dst, p_vm->hal.di_read(srcA));
                    } else if (flags == VM_IO_AI && p_vm->hal.ai_read != NULL) {
                        write_int(p_vm, dst, p_vm->hal.ai_read(srcA));
                    }
                }
                break;
            }
            case VM_OP_IO_WRITE: {
                if (enabled) {
                    if (flags == VM_IO_DO && p_vm->hal.do_write != NULL) {
                        p_vm->hal.do_write(dst, read_bool(p_vm, srcA));
                    } else if (flags == VM_IO_AO && p_vm->hal.ao_write != NULL) {
                        p_vm->hal.ao_write(dst, read_int(p_vm, srcA));
                    }
                }
                break;
            }
            case VM_OP_IO_PWM: {
                nextPc = pc + 12U;
                if (nextPc > p_vm->imageSizeBytes) { break; }
                uint16_t frqSlot = read_u16_le(&p_vm->imageBuf[pc + 8U]);
                uint16_t dtySlot = read_u16_le(&p_vm->imageBuf[pc + 10U]);
                if (p_vm->hal.pwm_set != NULL) {
                    float dty = enabled ? read_float(p_vm, dtySlot) : 0.0f;
                    uint8_t act = p_vm->hal.pwm_set(srcA, read_int(p_vm, frqSlot), dty);
                    write_bool(p_vm, dst, act);
                }
                break;
            }
            default: {
                p_vm->lastFaultCode = VM_ERR_FAULT_OPCODE;
                p_vm->lastFaultPc   = (uint16_t)pc;
                p_vm->runState      = VM_STATE_FAULT;
                return VM_ERR_FAULT_OPCODE;
            }
        }

        pc = nextPc;
    }

    p_vm->scanCount++;
    if (p_vm->hal.time_micros != NULL) {
        p_vm->threadActual_us[threadId] = p_vm->hal.time_micros() - start_us;
    }
    return VM_OK;
}
