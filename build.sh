#!/usr/bin/env sh
colcon build \
  --merge-install \
  --event-handlers console_cohesion+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -GNinja \
    -DQT_HOST_PATH="/opt/mamba/envs/morai" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    -DBUILD_TESTS=OFF \
    -DBUILD_TESTS_PCL=OFF \
    -DCMAKE_CXX_STANDARD=17 \
    $@
