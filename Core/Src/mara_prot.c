// Portable, transport-agnostic MaraProt v1 protocol parser and command dispatcher.

#include "mara_prot.h"
#include <string.h>

#define PROT_CRC16_POLY          0x1021U
#define PROT_CRC16_INIT          0xFFFFU
#define PROT_MODEL_NAME          "Duino - S"
#define PROT_MODEL_NAME_BYTES    24U
#define PROT_CTRL_STOP           0U
#define PROT_CTRL_START          1U
#define PROT_CTRL_PAUSE          2U
#define PROT_CTRL_ESTOP_RESET    5U
#define PROT_CTRL_CLEAR_FAULT    6U

static void write_u16_le(uint8_t *p_buf, uint16_t val)
{
    p_buf[0] = (uint8_t)(val & 0xFFU);
    p_buf[1] = (uint8_t)((val >> 8U) & 0xFFU);
}

static void write_u32_le(uint8_t *p_buf, uint32_t val)
{
    p_buf[0] = (uint8_t)(val & 0xFFU);
    p_buf[1] = (uint8_t)((val >> 8U) & 0xFFU);
    p_buf[2] = (uint8_t)((val >> 16U) & 0xFFU);
    p_buf[3] = (uint8_t)((val >> 24U) & 0xFFU);
}

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

static void send_response(ProtInstance_t *p_prot, uint16_t seqId, uint8_t opcode,
                          ProtStatus_t status, uint16_t payloadLen)
{
    if (p_prot->transport.tx_send == NULL || payloadLen > PROT_MAX_PAYLOAD_BYTES) {
        return;
    }

    uint8_t verFlags = PROT_VERSION_1 | PROT_FLAG_RESPONSE;
    if (status != PROT_STATUS_OK) {
        verFlags |= PROT_FLAG_ERROR;
    }

    p_prot->txBuf[0] = PROT_MAGIC_0;
    p_prot->txBuf[1] = PROT_MAGIC_1;
    p_prot->txBuf[2] = verFlags;
    p_prot->txBuf[3] = p_prot->transport.stationId;
    write_u16_le(&p_prot->txBuf[4], seqId);
    p_prot->txBuf[6] = opcode;
    p_prot->txBuf[7] = (uint8_t)status;
    write_u16_le(&p_prot->txBuf[8], payloadLen);

    uint16_t crcPos = (uint16_t)(PROT_HEADER_BYTES + payloadLen);
    uint16_t crc    = PROT_CalcCrc16(p_prot->txBuf, crcPos);
    write_u16_le(&p_prot->txBuf[crcPos], crc);

    p_prot->transport.tx_send(p_prot->txBuf, (uint16_t)(crcPos + PROT_CRC_BYTES));
}

static void handle_ping(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    uint32_t echoVal   = (reqLen >= 4U) ? read_u32_le(p_req) : 0U;
    uint32_t uptime_ms = (p_prot->transport.get_uptime_ms != NULL) ? p_prot->transport.get_uptime_ms() : 0U;

    write_u32_le(&p_prot->txBuf[PROT_HEADER_BYTES + 0U], echoVal);
    write_u32_le(&p_prot->txBuf[PROT_HEADER_BYTES + 4U], uptime_ms);
    send_response(p_prot, seqId, PROT_OP_PING, PROT_STATUS_OK, 8U);
}

static void handle_get_info(ProtInstance_t *p_prot, uint16_t seqId)
{
    uint8_t *p_out = &p_prot->txBuf[PROT_HEADER_BYTES];
    memset(p_out, 0, 44U);

    write_u16_le(&p_out[0], 0x0001U);
    write_u16_le(&p_out[2], p_prot->transport.modelId);
    write_u32_le(&p_out[4], p_prot->transport.fwVersion);
    p_out[8] = (uint8_t)p_prot->transport.kind;
    write_u16_le(&p_out[9], p_prot->transport.maxMtuBytes);
    p_out[11] = (uint8_t)VM_GetState(p_prot->p_vm);
    write_u32_le(&p_out[12], p_prot->p_vm->hasValidImage ? p_prot->p_vm->header.payloadCrc32 : 0U);
    write_u32_le(&p_out[16], p_prot->p_vm->hasValidImage ? p_prot->p_vm->header.projectHash : 0U);
    strncpy((char *)&p_out[20], PROT_MODEL_NAME, PROT_MODEL_NAME_BYTES - 1U);

    send_response(p_prot, seqId, PROT_OP_GET_INFO, PROT_STATUS_OK, 44U);
}

static void handle_vm_status(ProtInstance_t *p_prot, uint16_t seqId)
{
    uint8_t *p_out = &p_prot->txBuf[PROT_HEADER_BYTES];
    p_out[0] = (uint8_t)p_prot->p_vm->runState;
    p_out[1] = p_prot->p_vm->estopActive;
    write_u16_le(&p_out[2], p_prot->p_vm->estopMask);
    write_u32_le(&p_out[4], p_prot->p_vm->scanCount);
    for (uint32_t i = 0U; i < 5U; i++) {
        write_u32_le(&p_out[8U + (i * 4U)], p_prot->p_vm->threadActual_us[i + 1U]);
    }
    write_u16_le(&p_out[28], (uint16_t)p_prot->p_vm->lastFaultCode);
    write_u16_le(&p_out[30], p_prot->p_vm->lastFaultPc);

    send_response(p_prot, seqId, PROT_OP_VM_STATUS, PROT_STATUS_OK, 32U);
}

static void handle_vm_control(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 1U) {
        send_response(p_prot, seqId, PROT_OP_VM_CONTROL, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint8_t    action = p_req[0];
    VmStatus_t vmSt   = VM_OK;
    if (action == PROT_CTRL_STOP) {
        vmSt = VM_SetState(p_prot->p_vm, VM_STATE_STOPPED);
        if (vmSt == VM_OK) { vmSt = VM_Reset(p_prot->p_vm); }
    } else if (action == PROT_CTRL_START) {
        vmSt = VM_SetState(p_prot->p_vm, VM_STATE_RUNNING);
    } else if (action == PROT_CTRL_PAUSE) {
        vmSt = VM_SetState(p_prot->p_vm, VM_STATE_PAUSED);
    } else if (action == PROT_CTRL_ESTOP_RESET) {
        p_prot->p_vm->estopActive = 0U;
        p_prot->p_vm->estopMask   = 0U;
    } else if (action == PROT_CTRL_CLEAR_FAULT) {
        p_prot->p_vm->lastFaultCode = VM_OK;
        p_prot->p_vm->runState      = VM_STATE_STOPPED;
    } else {
        send_response(p_prot, seqId, PROT_OP_VM_CONTROL, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    ProtStatus_t respSt = (vmSt == VM_OK) ? PROT_STATUS_OK : PROT_STATUS_ERR_STATE;
    p_prot->txBuf[PROT_HEADER_BYTES + 0U] = (uint8_t)p_prot->p_vm->runState;
    p_prot->txBuf[PROT_HEADER_BYTES + 1U] = p_prot->p_vm->estopActive;
    send_response(p_prot, seqId, PROT_OP_VM_CONTROL, respSt, 2U);
}

static void handle_upload_begin(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 8U) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_BEGIN, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint32_t totalBytes = read_u32_le(&p_req[0]);
    uint32_t crc32      = read_u32_le(&p_req[4]);
    if (totalBytes < VM_HEADER_SIZE_BYTES || totalBytes > VM_MAX_IMAGE_BYTES) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_BEGIN, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    VM_SetState(p_prot->p_vm, VM_STATE_STOPPED);
    p_prot->p_vm->hasValidImage = false;
    p_prot->uploadTotalBytes      = totalBytes;
    p_prot->uploadExpectedCrc32   = crc32;
    p_prot->uploadNextOffset      = 0U;
    p_prot->uploadActive          = true;

    uint16_t maxChunk = (uint16_t)(p_prot->transport.maxMtuBytes - 4U);
    write_u16_le(&p_prot->txBuf[PROT_HEADER_BYTES], maxChunk);
    send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_BEGIN, PROT_STATUS_OK, 2U);
}

static void handle_upload_chunk(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (!p_prot->uploadActive || reqLen < 5U) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_CHUNK, PROT_STATUS_ERR_STATE, 0U);
        return;
    }

    uint32_t offset   = read_u32_le(&p_req[0]);
    uint16_t dataLen  = (uint16_t)(reqLen - 4U);
    if ((offset + dataLen) > p_prot->uploadTotalBytes || offset > p_prot->uploadNextOffset) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_CHUNK, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    // Idempotent retry: if offset already received, ACK current nextOffset
    if (offset == p_prot->uploadNextOffset) {
        memcpy(&p_prot->p_vm->imageBuf[offset], &p_req[4], dataLen);
        if (p_prot->transport.storage_write != NULL) {
            if (!p_prot->transport.storage_write(offset, &p_req[4], dataLen)) {
                send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_CHUNK, PROT_STATUS_ERR_STORAGE, 0U);
                return;
            }
        }
        p_prot->uploadNextOffset += dataLen;
    }

    write_u32_le(&p_prot->txBuf[PROT_HEADER_BYTES], p_prot->uploadNextOffset);
    send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_CHUNK, PROT_STATUS_OK, 4U);
}

static void handle_upload_commit(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (!p_prot->uploadActive || p_prot->uploadNextOffset != p_prot->uploadTotalBytes) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_COMMIT, PROT_STATUS_ERR_STATE, 0U);
        return;
    }

    p_prot->uploadActive = false;
    uint32_t fullCrc = VM_CalcCrc32(p_prot->p_vm->imageBuf, p_prot->uploadTotalBytes);
    if (fullCrc != p_prot->uploadExpectedCrc32) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_COMMIT, PROT_STATUS_ERR_CRC, 0U);
        return;
    }

    // Validate MFLW header and payload CRC in-place inside imageBuf
    VmStatus_t st = VM_LoadImage(p_prot->p_vm, p_prot->p_vm->imageBuf, p_prot->uploadTotalBytes);
    if (st != VM_OK) {
        send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_COMMIT, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    if (reqLen >= 1U && p_req[0] != 0U) {
        VM_SetState(p_prot->p_vm, VM_STATE_RUNNING);
    }

    write_u32_le(&p_prot->txBuf[PROT_HEADER_BYTES], p_prot->p_vm->header.payloadCrc32);
    p_prot->txBuf[PROT_HEADER_BYTES + 4U] = (uint8_t)p_prot->p_vm->runState;
    send_response(p_prot, seqId, PROT_OP_VM_UPLOAD_COMMIT, PROT_STATUS_OK, 5U);
}

static void handle_watch_subscribe(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 3U) {
        send_response(p_prot, seqId, PROT_OP_WATCH_SUBSCRIBE, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint8_t  mode     = p_req[0];
    uint16_t count    = read_u16_le(&p_req[1]);
    uint16_t startIdx = (mode == 1U) ? p_prot->watchCount : 0U;
    if ((startIdx + count) > PROT_MAX_WATCH_SLOTS || (3U + (count * 2U)) > reqLen) {
        send_response(p_prot, seqId, PROT_OP_WATCH_SUBSCRIBE, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    p_prot->watchCount = (uint16_t)(startIdx + count);
    p_prot->watchEpoch = 1U;
    for (uint16_t i = 0U; i < count; i++) {
        uint16_t dstIdx = (uint16_t)(startIdx + i);
        p_prot->watchSlots[dstIdx] = read_u16_le(&p_req[3U + (i * 2U)]);
        memset(p_prot->watchLastRaw[dstIdx], 0, VM_MAX_SLOT_BYTES);
    }

    write_u16_le(&p_prot->txBuf[PROT_HEADER_BYTES + 0U], p_prot->watchCount);
    write_u16_le(&p_prot->txBuf[PROT_HEADER_BYTES + 2U], p_prot->watchEpoch);
    send_response(p_prot, seqId, PROT_OP_WATCH_SUBSCRIBE, PROT_STATUS_OK, 4U);
}

static void handle_watch_poll(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    uint16_t ackEpoch   = (reqLen >= 2U) ? read_u16_le(&p_req[0]) : 0U;
    bool     forceFull  = (ackEpoch == 0U || ackEpoch != p_prot->watchEpoch);
    uint8_t *p_out      = &p_prot->txBuf[PROT_HEADER_BYTES];
    uint16_t pos        = 6U;
    uint16_t emitted    = 0U;
    uint16_t maxPayload = p_prot->transport.maxMtuBytes;
    if (maxPayload > PROT_MAX_PAYLOAD_BYTES) {
        maxPayload = PROT_MAX_PAYLOAD_BYTES;
    }

    uint8_t curRaw[VM_MAX_SLOT_BYTES];
    for (uint16_t i = 0U; i < p_prot->watchCount; i++) {
        uint16_t slotId = p_prot->watchSlots[i];
        uint8_t  valLen = VM_ReadSlotRaw(p_prot->p_vm, slotId, curRaw);
        if (valLen == 0U) {
            continue;
        }
        if (!forceFull && memcmp(curRaw, p_prot->watchLastRaw[i], valLen) == 0) {
            continue;
        }
        if ((pos + 2U + valLen) > maxPayload) {
            break;
        }
        write_u16_le(&p_out[pos], slotId);
        memcpy(&p_out[pos + 2U], curRaw, valLen);
        memcpy(p_prot->watchLastRaw[i], curRaw, valLen);
        pos += (uint16_t)(2U + valLen);
        emitted++;
    }

    p_prot->watchEpoch++;
    if (p_prot->watchEpoch == 0U) {
        p_prot->watchEpoch = 1U;
    }

    write_u16_le(&p_out[0], p_prot->watchEpoch);
    p_out[2] = (uint8_t)p_prot->p_vm->runState;
    p_out[3] = p_prot->p_vm->estopActive;
    write_u16_le(&p_out[4], emitted);
    send_response(p_prot, seqId, PROT_OP_WATCH_POLL, PROT_STATUS_OK, pos);
}

static void handle_slot_read_batch(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 2U) {
        send_response(p_prot, seqId, PROT_OP_SLOT_READ_BATCH, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint16_t count   = read_u16_le(&p_req[0]);
    if ((2U + (count * 2U)) > reqLen) {
        send_response(p_prot, seqId, PROT_OP_SLOT_READ_BATCH, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint8_t *p_out   = &p_prot->txBuf[PROT_HEADER_BYTES];
    uint16_t pos     = 2U;
    uint16_t emitted = 0U;
    uint8_t  curRaw[VM_MAX_SLOT_BYTES];

    for (uint16_t i = 0U; i < count; i++) {
        uint16_t slotId = read_u16_le(&p_req[2U + (i * 2U)]);
        uint8_t  valLen = VM_ReadSlotRaw(p_prot->p_vm, slotId, curRaw);
        if (valLen == 0U || (pos + 2U + valLen) > p_prot->transport.maxMtuBytes) {
            break;
        }
        write_u16_le(&p_out[pos], slotId);
        memcpy(&p_out[pos + 2U], curRaw, valLen);
        pos += (uint16_t)(2U + valLen);
        emitted++;
    }

    write_u16_le(&p_out[0], emitted);
    send_response(p_prot, seqId, PROT_OP_SLOT_READ_BATCH, PROT_STATUS_OK, pos);
}

static void handle_slot_write_batch(ProtInstance_t *p_prot, uint16_t seqId, const uint8_t *p_req, uint16_t reqLen)
{
    if (reqLen < 2U) {
        send_response(p_prot, seqId, PROT_OP_SLOT_WRITE_BATCH, PROT_STATUS_ERR_PARAM, 0U);
        return;
    }

    uint16_t count   = read_u16_le(&p_req[0]);
    uint16_t pos     = 2U;
    uint16_t applied = 0U;

    for (uint16_t i = 0U; i < count; i++) {
        if ((pos + 2U) > reqLen) {
            break;
        }
        uint16_t slotId = read_u16_le(&p_req[pos]);
        uint8_t  valLen = VM_GetSlotSize(slotId);
        if (valLen == 0U || (pos + 2U + valLen) > reqLen) {
            break;
        }
        if (VM_WriteSlotRaw(p_prot->p_vm, slotId, &p_req[pos + 2U]) > 0U) {
            applied++;
        }
        pos += (uint16_t)(2U + valLen);
    }

    write_u16_le(&p_prot->txBuf[PROT_HEADER_BYTES], applied);
    send_response(p_prot, seqId, PROT_OP_SLOT_WRITE_BATCH, PROT_STATUS_OK, 2U);
}

static void dispatch_frame(ProtInstance_t *p_prot)
{
    uint8_t        stationId  = p_prot->rxBuf[3];
    uint16_t       seqId      = read_u16_le(&p_prot->rxBuf[4]);
    uint8_t        opcode     = p_prot->rxBuf[6];
    uint16_t       payloadLen = p_prot->rxExpectedPayload;
    const uint8_t *p_payload  = &p_prot->rxBuf[PROT_HEADER_BYTES];

    if (stationId != PROT_STATION_P2P &&
        stationId != PROT_STATION_BROADCAST &&
        stationId != p_prot->transport.stationId) {
        return;
    }

    uint16_t crcCalc = PROT_CalcCrc16(p_prot->rxBuf, (uint16_t)(PROT_HEADER_BYTES + payloadLen));
    uint16_t crcRx   = read_u16_le(&p_prot->rxBuf[PROT_HEADER_BYTES + payloadLen]);
    if (crcCalc != crcRx) {
        send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_CRC, 0U);
        return;
    }

    switch (opcode) {
        case PROT_OP_PING:             handle_ping(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_GET_INFO:         handle_get_info(p_prot, seqId); break;
        case PROT_OP_VM_STATUS:        handle_vm_status(p_prot, seqId); break;
        case PROT_OP_VM_CONTROL:       handle_vm_control(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_VM_UPLOAD_BEGIN:  handle_upload_begin(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_VM_UPLOAD_CHUNK:  handle_upload_chunk(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_VM_UPLOAD_COMMIT: handle_upload_commit(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_WATCH_SUBSCRIBE:  handle_watch_subscribe(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_WATCH_POLL:       handle_watch_poll(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_SLOT_READ_BATCH:  handle_slot_read_batch(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_SLOT_WRITE_BATCH: handle_slot_write_batch(p_prot, seqId, p_payload, payloadLen); break;
        case PROT_OP_RTC_GET: {
            if (p_prot->transport.rtc_get != NULL) {
                uint64_t raw = (uint64_t)p_prot->transport.rtc_get();
                for (uint8_t i = 0U; i < 8U; i++) {
                    p_prot->txBuf[PROT_HEADER_BYTES + i] = (uint8_t)((raw >> (i * 8U)) & 0xFFU);
                }
                send_response(p_prot, seqId, opcode, PROT_STATUS_OK, 8U);
            } else {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_OPCODE, 0U);
            }
            break;
        }
        case PROT_OP_RTC_SET: {
            if (payloadLen < 8U) {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_PARAM, 0U);
            } else if (p_prot->transport.rtc_set != NULL) {
                uint64_t raw = 0U;
                for (uint8_t i = 0U; i < 8U; i++) {
                    raw |= ((uint64_t)p_payload[i]) << (i * 8U);
                }
                bool ok = p_prot->transport.rtc_set((int64_t)raw);
                send_response(p_prot, seqId, opcode, ok ? PROT_STATUS_OK : PROT_STATUS_ERR_PARAM, 0U);
            } else {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_OPCODE, 0U);
            }
            break;
        }
        case PROT_OP_FW_ENTER_ROM_DFU: {
            if (payloadLen >= 4U && read_u32_le(p_payload) == PROT_DFU_MAGIC && p_prot->transport.enter_dfu != NULL) {
                p_prot->txBuf[PROT_HEADER_BYTES] = 1U;
                send_response(p_prot, seqId, opcode, PROT_STATUS_OK, 1U);
                p_prot->transport.enter_dfu();
            } else {
                send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_PARAM, 0U);
            }
            break;
        }
        default:
            send_response(p_prot, seqId, opcode, PROT_STATUS_ERR_OPCODE, 0U);
            break;
    }
}

uint16_t PROT_CalcCrc16(const uint8_t *p_data, uint16_t lenBytes)
{
    uint16_t crc = PROT_CRC16_INIT;
    for (uint16_t i = 0U; i < lenBytes; i++) {
        crc ^= (uint16_t)p_data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ PROT_CRC16_POLY);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

bool PROT_Init(ProtInstance_t *p_prot, VmInstance_t *p_vm, const ProtTransport_t *p_transport)
{
    if (p_prot == NULL || p_vm == NULL || p_transport == NULL) {
        return false;
    }
    memset(p_prot, 0, sizeof(*p_prot));
    p_prot->p_vm      = p_vm;
    p_prot->transport = *p_transport;
    if (p_prot->transport.maxMtuBytes == 0U || p_prot->transport.maxMtuBytes > PROT_MAX_PAYLOAD_BYTES) {
        p_prot->transport.maxMtuBytes = PROT_MAX_PAYLOAD_BYTES;
    }
    p_prot->rxState = PROT_RX_SOF0;
    return true;
}

void PROT_FeedByte(ProtInstance_t *p_prot, uint8_t byte)
{
    if (p_prot == NULL) {
        return;
    }

    switch (p_prot->rxState) {
        case PROT_RX_SOF0:
            if (byte == PROT_MAGIC_0) {
                p_prot->rxBuf[0] = byte;
                p_prot->rxState  = PROT_RX_SOF1;
            }
            break;

        case PROT_RX_SOF1:
            if (byte == PROT_MAGIC_1) {
                p_prot->rxBuf[1] = byte;
                p_prot->rxPos    = 2U;
                p_prot->rxState  = PROT_RX_HEADER;
            } else if (byte != PROT_MAGIC_0) {
                p_prot->rxState  = PROT_RX_SOF0;
            }
            break;

        case PROT_RX_HEADER:
            p_prot->rxBuf[p_prot->rxPos++] = byte;
            if (p_prot->rxPos >= PROT_HEADER_BYTES) {
                p_prot->rxExpectedPayload = read_u16_le(&p_prot->rxBuf[8]);
                if (p_prot->rxExpectedPayload > PROT_MAX_PAYLOAD_BYTES) {
                    p_prot->rxState = PROT_RX_SOF0;
                } else if (p_prot->rxExpectedPayload == 0U) {
                    p_prot->rxState = PROT_RX_CRC0;
                } else {
                    p_prot->rxState = PROT_RX_PAYLOAD;
                }
            }
            break;

        case PROT_RX_PAYLOAD:
            p_prot->rxBuf[p_prot->rxPos++] = byte;
            if (p_prot->rxPos >= (uint16_t)(PROT_HEADER_BYTES + p_prot->rxExpectedPayload)) {
                p_prot->rxState = PROT_RX_CRC0;
            }
            break;

        case PROT_RX_CRC0:
            p_prot->rxBuf[p_prot->rxPos++] = byte;
            p_prot->rxState = PROT_RX_CRC1;
            break;

        case PROT_RX_CRC1:
            p_prot->rxBuf[p_prot->rxPos++] = byte;
            p_prot->rxState = PROT_RX_SOF0;
            dispatch_frame(p_prot);
            break;

        default:
            p_prot->rxState = PROT_RX_SOF0;
            break;
    }
}

void PROT_FeedBuffer(ProtInstance_t *p_prot, const uint8_t *p_data, uint16_t lenBytes)
{
    if (p_prot == NULL || p_data == NULL) {
        return;
    }
    for (uint16_t i = 0U; i < lenBytes; i++) {
        PROT_FeedByte(p_prot, p_data[i]);
    }
}
