#!/bin/bash
cd /home/linux/rc2025/go2_runner
mkdir -p build
cd build
cmake .. 2>&1 | tee /home/linux/rc2025/go2_runner/cmake.log
make -j$(nproc) 2>&1 | tee /home/linux/rc2025/go2_runner/make.log
echo "DONE" > /home/linux/rc2025/go2_runner/BUILD_OK.txt
ls -la rc2025_run