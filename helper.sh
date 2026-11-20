#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

echo "==> Mass erasing STM32F407 flash via DAPLink..."
openocd \
  -f openocd.cfg \
  -c "init" \
  -c "reset halt" \
  -c "stm32f4x mass_erase 0" \
  -c "shutdown"

echo "==> Chip erased!"
