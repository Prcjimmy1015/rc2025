#!/bin/bash
set -e
echo "=== VERIFY SOURCE ==="
grep "TASK0_INLINE" /home/linux/rc2025/go2_runner/main.cpp || exit 1
echo "SOURCE OK"

echo "=== CLEAN BUILD ==="
rm -rf /home/linux/rc2025/go2_runner/build
mkdir -p /home/linux/rc2025/go2_runner/build
cd /home/linux/rc2025/go2_runner/build

echo "=== CMAKE ==="
cmake .. -G "Unix Makefiles"

echo "=== VERIFY TARGETS ==="
make -n rc2025_run 2>&1 | head -5

echo "=== MAKE (force all) ==="
make -j$(nproc) VERBOSE=0

echo "=== VERIFY BINARY ==="
if strings rc2025_run | grep -q "TASK0_INLINE"; then
    echo "SUCCESS: TASK0_INLINE found in binary"
else
    echo "FAILED: TASK0_INLINE NOT in binary - rebuilding main.o"
    rm -f CMakeFiles/rc2025_run.dir/main.cpp.o
    make -j$(nproc)
    strings rc2025_run | grep "TASK0_INLINE" && echo "RETRY SUCCESS" || echo "STILL FAILED"
fi

echo "=== BINARY INFO ==="
ls -la rc2025_run
echo ""
echo "Run: cd /home/linux/rc2025/go2_runner/build && LD_LIBRARY_PATH=/home/linux/unitree_sdk2/thirdparty/lib/x86_64:\$LD_LIBRARY_PATH ./rc2025_run ens37 --task 0"