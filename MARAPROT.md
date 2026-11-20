# MaraProt v1 Protocol Specification

MaraProt v1 is the binary request/response protocol used by **Duino - S** (`F407PLC`) over **USB CDC**, **Ethernet TCP/IP** (port `5020`), and **RS-485**. All multi-byte integers are **little-endian**.

Reference implementation:
- `Core/Inc/mara_prot.h`
- `Core/Src/mara_prot.c`
- `Core/Src/plc_hal.c`

---

## 1. Frame Format

Maximum payload size: `520` bytes (`PROT_MAX_PAYLOAD_BYTES`).  
Maximum frame size: `532` bytes (`10`-byte header + `520`-byte payload + `2`-byte CRC).

| Offset | Size | Field | Description |
| --- | --- | --- | --- |
| `0` | 1 | `magic0` | `0x4D` (`'M'`) |
| `1` | 1 | `magic1` | `0x50` (`'P'`) |
| `2` | 1 | `verFlags` | High nibble: protocol version `0x10` (v1). Low nibble flags: `0x01` = Response (`PROT_FLAG_RESPONSE`), `0x02` = Error (`PROT_FLAG_ERROR`). |
| `3` | 1 | `stationId` | Station address (`0x00` = P2P, `0xFF` = Broadcast, `1..254` = RS-485 unit ID). |
| `4` | 2 | `seqId` | Request sequence ID (`uint16_t`), echoed in response. |
| `6` | 1 | `opcode` | Command opcode (`ProtOpcode_t`). |
| `7` | 1 | `status` | Request: `0x00`. Response: `ProtStatus_t`. |
| `8` | 2 | `payloadLen` | Payload length in bytes (`uint16_t`, `0..520`). |
| `10` | `payloadLen` | `payload` | Command-specific payload bytes. |
| `10 + payloadLen` | 2 | `crc16` | CRC-16-CCITT (`poly = 0x1021`, `init = 0xFFFF`, no final XOR) computed over bytes `0 .. (9 + payloadLen)`. |

### Station ID Filtering
- On `PROT_TRANSPORT_RS485`: frames are processed only if `stationId == 0x00` (`PROT_STATION_P2P`), `stationId == 0xFF` (`PROT_STATION_BROADCAST`), or `stationId == transport.stationId`. Other frames are silently ignored.
- On `PROT_TRANSPORT_USB_CDC` and `PROT_TRANSPORT_TCP_IP` (point-to-point): any `stationId` is accepted.

---

## 2. Status Codes (`ProtStatus_t`)

| Code | Name | Meaning |
| --- | --- | --- |
| `0x00` | `PROT_STATUS_OK` | Command succeeded (`PROT_FLAG_ERROR` clear). |
| `0x01` | `PROT_STATUS_ERR_CRC` | Frame CRC-16 mismatch or upload image CRC-32 mismatch. |
| `0x02` | `PROT_STATUS_ERR_OPCODE` | Unsupported opcode or unavailable hardware callback. |
| `0x03` | `PROT_STATUS_ERR_PARAM` | Invalid payload length, parameter, or MFLW image header. |
| `0x04` | `PROT_STATUS_ERR_BUSY` | Target busy. |
| `0x05` | `PROT_STATUS_ERR_STATE` | Invalid VM or upload state for the requested operation. |
| `0x06` | `PROT_STATUS_ERR_STORAGE` | EEPROM/storage write failure. |

---

## 3. Transport Kinds (`ProtTransportKind_t`)

| Value | Name | Notes |
| --- | --- | --- |
| `0` | `PROT_TRANSPORT_USB_CDC` | USB Virtual COM port. |
| `1` | `PROT_TRANSPORT_TCP_IP` | Ethernet TCP server on port `5020` (10BASE-T, DHCP enabled by default). |
| `2` | `PROT_TRANSPORT_RS485` | Multidrop RS-485 serial bus. |

---

## 4. Opcodes (`ProtOpcode_t`)

### `0x01` — `PROT_OP_PING`
Connectivity and uptime check.
- **Request:** Optional `uint32_t echoVal` (4 bytes; defaults to `0` if omitted).
- **Response (`8` bytes):**
  - `[0..3]` `uint32_t echoVal`
  - `[4..7]` `uint32_t uptime_ms`

### `0x02` — `PROT_OP_GET_INFO`
Query device identity, transport MTU, and loaded program metadata.
- **Request:** Empty (`0` bytes).
- **Response (`44` bytes):**
  - `[0..1]` `uint16_t protoVersion` (`0x0001`)
  - `[2..3]` `uint16_t modelId`
  - `[4..7]` `uint32_t fwVersion`
  - `[8]` `uint8_t transportKind` (`ProtTransportKind_t`)
  - `[9..10]` `uint16_t maxMtuBytes`
  - `[11]` `uint8_t vmState` (`VmState_t`)
  - `[12..15]` `uint32_t imagePayloadCrc32` (`0` if no valid image)
  - `[16..19]` `uint32_t projectHash` (`0` if no valid image)
  - `[20..43]` `char[24]` null-terminated model name (`"Duino - S"`)

### `0x10` — `PROT_OP_VM_STATUS`
Query VM runtime state, E-STOP status, scan counters, and fault info.
- **Request:** Empty (`0` bytes).
- **Response (`32` bytes):**
  - `[0]` `uint8_t runState`
  - `[1]` `uint8_t estopActive`
  - `[2..3]` `uint16_t estopMask`
  - `[4..7]` `uint32_t scanCount`
  - `[8..27]` `uint32_t[5] threadActual_us` (threads 1..5 execution time in microseconds)
  - `[28..29]` `uint16_t lastFaultCode`
  - `[30..31]` `uint16_t lastFaultPc`

### `0x11` — `PROT_OP_VM_CONTROL`
Control VM execution or clear E-STOP/faults.
- **Request (`1` byte):**
  - `[0]` `uint8_t action`:
    - `0` (`PROT_CTRL_STOP`): Stop and reset VM
    - `1` (`PROT_CTRL_START`): Start VM
    - `2` (`PROT_CTRL_PAUSE`): Pause VM
    - `5` (`PROT_CTRL_ESTOP_RESET`): Clear E-STOP flags
    - `6` (`PROT_CTRL_CLEAR_FAULT`): Clear last fault code and set state to stopped
- **Response (`2` bytes):**
  - `[0]` `uint8_t runState`
  - `[1]` `uint8_t estopActive`

### `0x20` — `PROT_OP_VM_UPLOAD_BEGIN`
Begin uploading an `MFLW` bytecode image (stops VM and clears current valid-image flag).
- **Request (`8` bytes):**
  - `[0..3]` `uint32_t totalBytes` (`VM_HEADER_SIZE_BYTES .. VM_MAX_IMAGE_BYTES`)
  - `[4..7]` `uint32_t expectedCrc32` (CRC-32 over full image)
- **Response (`2` bytes):**
  - `[0..1]` `uint16_t maxChunkBytes` (`maxMtuBytes - 4`)

### `0x21` — `PROT_OP_VM_UPLOAD_CHUNK`
Write sequential bytecode chunk (idempotent retry supported if `offset < uploadNextOffset`).
- **Request (`4 + N` bytes):**
  - `[0..3]` `uint32_t offset`
  - `[4..4+N-1]` `uint8_t[N]` chunk bytes
- **Response (`4` bytes):**
  - `[0..3]` `uint32_t nextOffset`

### `0x22` — `PROT_OP_VM_UPLOAD_COMMIT`
Verify full image CRC-32, validate `MFLW` header/image, and optionally start VM.
- **Request (`0` or `1` byte):**
  - `[0]` (optional) `uint8_t autoStart` (`!= 0` starts VM immediately)
- **Response (`5` bytes):**
  - `[0..3]` `uint32_t payloadCrc32`
  - `[4]` `uint8_t runState`

### `0x30` — `PROT_OP_WATCH_SUBSCRIBE`
Configure slot watch list (up to `512` slots, `PROT_MAX_WATCH_SLOTS`).
- **Request (`3 + 2 * count` bytes):**
  - `[0]` `uint8_t mode`: `0` = replace list, `1` = append to existing list
  - `[1..2]` `uint16_t count`
  - `[3..]` `uint16_t[count]` slot IDs
- **Response (`4` bytes):**
  - `[0..1]` `uint16_t totalWatchCount`
  - `[2..3]` `uint16_t watchEpoch` (`1`)

### `0x31` — `PROT_OP_WATCH_POLL`
Poll changed (or all, if `ackEpoch == 0` or `ackEpoch != currentEpoch`) subscribed slots.
- **Request (`0` or `2` bytes):**
  - `[0..1]` (optional) `uint16_t ackEpoch` (`0` forces full snapshot)
- **Response (`6 + ...` bytes, capped at `maxMtuBytes`):**
  - `[0..1]` `uint16_t newEpoch`
  - `[2]` `uint8_t runState`
  - `[3]` `uint8_t estopActive`
  - `[4..5]` `uint16_t emittedCount`
  - `[6..]` repeated `emittedCount` entries of `{ uint16_t slotId, uint8_t rawVal[slotSize] }`

### `0x32` — `PROT_OP_SLOT_READ_BATCH`
Read an arbitrary list of VM slots in one frame.
- **Request (`2 + 2 * count` bytes):**
  - `[0..1]` `uint16_t count`
  - `[2..]` `uint16_t[count]` slot IDs
- **Response (`2 + ...` bytes, capped at `maxMtuBytes`):**
  - `[0..1]` `uint16_t emittedCount`
  - `[2..]` repeated `emittedCount` entries of `{ uint16_t slotId, uint8_t rawVal[slotSize] }`

### `0x33` — `PROT_OP_SLOT_WRITE_BATCH`
Write values to an arbitrary list of VM slots in one frame.
- **Request (`2 + ...` bytes):**
  - `[0..1]` `uint16_t count`
  - `[2..]` repeated `count` entries of `{ uint16_t slotId, uint8_t rawVal[slotSize] }`
- **Response (`2` bytes):**
  - `[0..1]` `uint16_t appliedCount`

### `0x40` — `PROT_OP_RTC_GET`
Read PLC hardware RTC time as signed 64-bit Unix seconds.
- **Request:** Empty (`0` bytes).
- **Response (`8` bytes):**
  - `[0..7]` `int64_t unixSec`

### `0x41` — `PROT_OP_RTC_SET`
Set PLC hardware RTC time.
- **Request (`8` bytes):**
  - `[0..7]` `int64_t unixSec`
- **Response:** Empty (`0` bytes).

### `0x50` — `PROT_OP_ONEWIRE_SCAN`
Scan a 1-Wire bus on the given DIO pin for DS18B20/1-Wire ROM IDs (up to 8 sensors).
- **Request (`0` or `1` byte):**
  - `[0]` (optional) `uint8_t dioPin` (defaults to `13`)
- **Response (`1 + 8 * count` bytes):**
  - `[0]` `uint8_t count` (`0..8`)
  - `[1..]` `count` consecutive 8-byte ROM addresses

### `0xF3` — `PROT_OP_FW_ENTER_ROM_DFU`
Send ACK and jump into the STM32F407 internal ROM USB DFU bootloader (`0x1FFF0000`).
- **Request (`4` bytes):**
  - `[0..3]` `uint32_t magic` (`0xDF00B007`, `PROT_DFU_MAGIC`)
- **Response (`1` byte, sent prior to reset/jump):**
  - `[0]` `uint8_t accepted` (`1`)
