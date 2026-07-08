#!/bin/bash
cd /home/linux/rc2025/go2_runner
rm -rf build
mkdir build
cd build
cmake .. && make -j$(nproc)
echo "===== DONE ====="
ls -la rc2025_run