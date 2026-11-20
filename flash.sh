#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PRESET="${1:-Debug}"
BUILD_DIR="${SCRIPT_DIR}/build/${PRESET}"
ELF_FILE="${BUILD_DIR}/F407PLC.elf"

cd "${SCRIPT_DIR}"

if [[ ! -d "${BUILD_DIR}" ]]; then
  echo "==> Configuring CMake preset: ${PRESET}"
  cmake --preset "${PRESET}"
fi

echo "==> Building (${PRESET})..."
cmake --build --preset "${PRESET}"

if [[ ! -f "${ELF_FILE}" ]]; then
  echo "Error: ELF not found at ${ELF_FILE}" >&2
  exit 1
fi

echo "==> Flashing ${ELF_FILE} via DAPLink (CMSIS-DAP)..."
openocd \
  -f interface/cmsis-dap.cfg \
  -f target/stm32f4x.cfg \
  -c "adapter speed 1000" \
  -c "reset_config none" \
  -c "program ${ELF_FILE} verify reset exit"

echo "==> Done!"
