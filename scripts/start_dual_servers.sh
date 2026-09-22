#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sdk_dir="$(cd -- "${script_dir}/.." && pwd)"
server="${sdk_dir}/build/hps6axis_server"

usage() {
  echo "Usage: $0 <left-ttyUSB-number> <right-ttyUSB-number> [bind-address]" >&2
  echo "Example: $0 1 0 0.0.0.0  # left=/dev/ttyUSB1, right=/dev/ttyUSB0" >&2
}

if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage
  exit 2
fi

left_number="$1"
right_number="$2"
bind_address="${3:-0.0.0.0}"

if [[ ! "${left_number}" =~ ^[0-9]+$ || ! "${right_number}" =~ ^[0-9]+$ ]]; then
  echo "USB numbers must be non-negative integers." >&2
  usage
  exit 2
fi
if [[ "${left_number}" == "${right_number}" ]]; then
  echo "Left and right sensors cannot use the same ttyUSB number." >&2
  exit 2
fi

left_device="/dev/ttyUSB${left_number}"
right_device="/dev/ttyUSB${right_number}"

if [[ ! -x "${server}" ]]; then
  echo "Missing ${server}; run: cmake --build build -j\$(nproc)" >&2
  exit 1
fi

if [[ ! -e "${left_device}" ]]; then
  echo "Left sensor serial device does not exist: ${left_device}" >&2
  exit 1
fi
if [[ ! -e "${right_device}" ]]; then
  echo "Right sensor serial device does not exist: ${right_device}" >&2
  exit 1
fi

"${server}" "${left_device}" 9000 "${bind_address}" &
left_pid=$!
"${server}" "${right_device}" 9001 "${bind_address}" &
right_pid=$!

cleanup() {
  kill -TERM "${left_pid}" "${right_pid}" 2>/dev/null || true
  wait "${left_pid}" "${right_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait -n "${left_pid}" "${right_pid}"
