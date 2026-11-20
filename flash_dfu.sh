#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PRESET="${1:-Debug}"
BUILD_DIR="${SCRIPT_DIR}/build/${PRESET}"
BIN_FILE="${BUILD_DIR}/F407PLC.bin"

cd "${SCRIPT_DIR}"

if [[ ! -d "${BUILD_DIR}" ]]; then
  echo "==> Configuring CMake preset: ${PRESET}"
  cmake --preset "${PRESET}"
fi

echo "==> Building (${PRESET})..."
cmake --build --preset "${PRESET}"

if [[ ! -f "${BIN_FILE}" ]]; then
  echo "Error: BIN not found at ${BIN_FILE}" >&2
  exit 1
fi

if command -v dfu-util >/dev/null 2>&1; then
  echo "==> Waiting for STM32 ROM DFU device (0483:df11)..."
  for _ in $(seq 1 20); do
    if dfu-util -l 2>/dev/null | grep -qi "0483:df11"; then
      break
    fi
    sleep 0.5
  done
  echo "==> Flashing ${BIN_FILE} via dfu-util..."
  dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D "${BIN_FILE}"
elif command -v STM32_Programmer_CLI >/dev/null 2>&1; then
  echo "==> Flashing ${BIN_FILE} via STM32_Programmer_CLI (USB DFU)..."
  STM32_Programmer_CLI -c port=USB1 -w "${BIN_FILE}" 0x08000000 -v -s
else
  echo "Error: Neither dfu-util nor STM32_Programmer_CLI found in PATH." >&2
  echo "Install dfu-util (macOS: brew install dfu-util, Linux: sudo apt install dfu-util)" >&2
  exit 1
fi

echo "==> Done!"
