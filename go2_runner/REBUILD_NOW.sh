#!/bin/bash
set -e
cd /home/linux/rc2025/go2_runner

echo "=== verify source ==="
grep "TASK0_SAFE" cases/case0.cpp || { echo "SOURCE MISSING TASK0_SAFE!"; exit 1; }
echo "SOURCE OK"

echo "=== purge build ==="
rm -rf build
mkdir build && cd build

echo "=== cmake ==="
cmake .. -G "Unix Makefiles" > /dev/null

echo "=== force rebuild ==="
make -j$(nproc) 2>&1 | grep -E "case0|Linking|error"

echo "=== verify binary ==="
if strings rc2025_run | grep -q "TASK0_SAFE"; then
    echo "SUCCESS! TASK0_SAFE found in binary"
else
    echo "FAILED - trying manual case0 rebuild"
    rm -f CMakeFiles/rc2025_run.dir/cases/case0.cpp.o
    make -j$(nproc)
    strings rc2025_run | grep -q "TASK0_SAFE" && echo "RETRY OK" || echo "STILL FAILED"
fi

ls -la rc2025_run
echo ""
echo "=== RUN: ==="
echo "cd /home/linux/rc2025/go2_runner/build && LD_LIBRARY_PATH=/home/linux/unitree_sdk2/thirdparty/lib/x86_64:\$LD_LIBRARY_PATH ./rc2025_run ens37 --task 0"