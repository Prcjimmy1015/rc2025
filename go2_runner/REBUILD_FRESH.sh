#!/bin/bash
set -e
cd /home/linux/rc2025/go2_runner
echo "=== CHECKING SOURCE ==="
grep -n "TASK0_V2" cases/case0.cpp || { echo "ERROR: TASK0_V2 not in source!"; exit 1; }
echo "=== REMOVING BUILD ==="
rm -rf build
mkdir build
cd build
echo "=== CMAKE ==="
cmake .. -G "Unix Makefiles"
echo "=== MAKE CLEAN ==="
make clean 2>/dev/null || true
echo "=== MAKE ==="
make -j$(nproc)
echo "=== VERIFY BINARY ==="
strings rc2025_run | grep "TASK0" && echo "SUCCESS: new code in binary" || echo "FAIL: new code NOT in binary"
ls -la rc2025_run