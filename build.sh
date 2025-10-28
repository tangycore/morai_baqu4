#!/usr/bin/env bash
set -euo pipefail

: "${BUILD_TYPE:=Release}"
: "${JOBS:=$(nproc)}"

if [ $# -gt 0 ]; then
  # 특정 패키지 + 의존성 빌드
  echo "[build] Building package '$1' with dependencies..."
  catkin_make \
    --only-pkg-with-deps "$1" \
    -j${JOBS} \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_TESTING=OFF
else
  # 전체 빌드
  echo "[build] Building all packages..."
  catkin_make \
    -j${JOBS} \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_TESTING=OFF
fi

echo "[build] done"