#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$1" ]; then
    echo "Usage: $0 <ethernet_if>"
    echo "Example: $0 eth0"
    exit 1
fi

cd "${SCRIPT_DIR}/build"
cmake ..
make -j$(nproc)

echo "==> Running rc2025_run on interface: $1"
"${SCRIPT_DIR}/build/rc2025_run" "$1"
