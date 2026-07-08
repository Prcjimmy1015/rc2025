#!/bin/bash
set -e
DIR=/home/linux/rc2025/go2_runner
echo "==> 清理旧 build..."
rm -rf "$DIR/build"
mkdir -p "$DIR/build"
cd "$DIR/build"

echo "==> cmake..."
cmake ..

echo "==> make..."
make -j$(nproc)
echo "==> 构建完成！"

echo ""
echo "==============================================="
echo "  测试命令:"
echo "  pure line follow:"
echo "    $DIR/build/rc2025_run ens37 --task 0"
echo ""
echo "  full auto:"
echo "    $DIR/build/rc2025_run ens37"
echo "==============================================="