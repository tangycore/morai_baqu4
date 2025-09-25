#!/usr/bin/env bash
set -euo pipefail

: "${BUILD_TYPE:=Release}"
: "${JOBS:=$(nproc)}"

if [ $# -gt 0 ]; then
  export CATKIN_WHITELIST_PACKAGES="$1"
else
  unset CATKIN_WHITELIST_PACKAGES 2>/dev/null || true
fi

catkin_make -j${JOBS} -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=OFF

echo "[build] done"