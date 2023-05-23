#! /bin/bash
#env
export ARCH=arm64
export SUBARCH=arm64
export PATH="$HOME/Toolchain/clang-r383902b/bin:$HOME/Toolchain/gcc64/bin:$HOME/Toolchain/gcc32/bin:$PATH"

args="-j$(8 --all) \
ARCH=arm64 \
SUBARCH=arm64 \
O=out \
CC=clang \
CROSS_COMPILE=aarch64-linux-android- \
CROSS_COMPILE_ARM32=arm-linux-androideabi- \
CLANG_TRIPLE=aarch64-linux-gnu- "

#clean
#build
make ${args} vendor/lito_defconfig
#make ${args} xxxx_defconfig
make Image ${args} 2>&1 | tee kernel.log

