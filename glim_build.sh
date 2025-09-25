#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# 기본값
: "${BUILD_TYPE:=Release}"
: "${JOBS:=$(nproc)}"

# --- 로컬 Eigen 경로 고정 ---
export EIGEN_LOCAL="$WS_DIR/src/slam/eigen"
if [ ! -d "$EIGEN_LOCAL/Eigen" ]; then
  echo "[ERR] $EIGEN_LOCAL/Eigen not found" >&2
  exit 1
fi

# 시스템 Eigen 차단 + 로컬 우선 인클루드
export CMAKE_IGNORE_PATH="/usr/include/eigen3${CMAKE_IGNORE_PATH:+:$CMAKE_IGNORE_PATH}"
export CXXFLAGS="-I$EIGEN_LOCAL ${CXXFLAGS:-}"
export CPPFLAGS="-I$EIGEN_LOCAL ${CPPFLAGS:-}"
unset PKG_CONFIG_PATH CMAKE_PREFIX_PATH CMAKE_MODULE_PATH

# --- CMake 우선순위: user -> /usr/local -> /usr ---
export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:$PATH"

# --- ROS Noetic ---
if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/noetic/setup.bash
  echo "[INFO] Loaded /opt/ros/noetic/setup.bash"
else
  echo "[ERR] /opt/ros/noetic/setup.bash not found." >&2
  exit 1
fi

# --- env.sh 부트스트랩 ---
mkdir -p install_isolated
if [ ! -f install_isolated/env.sh ]; then
  cat > install_isolated/env.sh <<'EOF'
#!/usr/bin/env bash
export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:$PATH"
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
exec "$@"
EOF
  chmod +x install_isolated/env.sh
fi
if ! grep -q '\$HOME/.local/bin' install_isolated/env.sh; then
  sed -i '1i export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:$PATH"' install_isolated/env.sh
fi

# --- (A) 슬램 묶음: isolated + 로컬 Eigen ---
PKGS=(eigen gtsam gtsam_points glim glim_ros)
SLAM_PKGS=("${PKGS[@]}")   # ★ 아래 단계(B)에서 사용할 변수 정의

# 필요시 rosdep
if command -v rosdep >/dev/null 2>&1; then
  echo "[INFO] rosdep install ..."
  rosdep install --from-paths src --ignore-src -r -y || true
fi

cmv="$(cmake --version | awk 'NR==1{print $3}')"
echo "[INFO] Using CMake ${cmv}"

CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  -DCMAKE_EXPORT_COMPILE_COMMANDS=1
  -DCMAKE_CXX_STANDARD=17
  # 로컬 Eigen 강제
  -DGTSAM_USE_SYSTEM_EIGEN=ON
  -DEIGEN3_INCLUDE_DIR="$EIGEN_LOCAL"
  # 기타
  -DBUILD_TESTING=OFF
  -DBUILD_TESTS=OFF
  -DBUILD_TESTS_PCL=OFF
  -DBUILD_DEMO=OFF
  -DBUILD_TOOLS=OFF
  -DBUILD_WITH_TBB=OFF
  -DGTSAM_WITH_TBB=OFF
  -DBUILD_WITH_OPENMP=ON
  -DBUILD_WITH_CUDA=OFF
  -DBUILD_WITH_CUDA_MULTIARCH=OFF
  -DBUILD_WITH_VIEWER=OFF
  -DBUILD_WITH_MARCH_NATIVE=ON
)

echo "[INFO] Building (isolated): ${PKGS[*]}"
catkin_make_isolated --install -j"$JOBS" \
  --pkg "${PKGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}"

# 슬램 오버레이 적용
# shellcheck source=/dev/null
source "$WS_DIR/install_isolated/setup.bash"

# ---- (B) 나머지 전체 패키지: 일반 catkin_make ----
# 로컬 Eigen 강제 설정은 여기선 해제(불필요 영향 방지)
unset CMAKE_IGNORE_PATH CXXFLAGS CPPFLAGS
unset PKG_CONFIG_PATH CMAKE_MODULE_PATH

if ! command -v catkin_topological_order >/dev/null 2>&1; then
  echo "[ERR] catkin_topological_order not found" >&2
  exit 1
fi

mapfile -t ALL_PKGS < <(catkin_topological_order "$WS_DIR/src" --only-names)

declare -A EX=()
for p in "${SLAM_PKGS[@]}"; do EX["$p"]=1; done

WHITELIST=()
for p in "${ALL_PKGS[@]}"; do
  if [[ -z "${EX[$p]:-}" ]]; then
    WHITELIST+=("$p")
  fi
done

if [ ${#WHITELIST[@]} -eq 0 ]; then
  echo "[INFO] [B] Nothing to build (rest empty)."
else
  IFS=";"; export CATKIN_WHITELIST_PACKAGES="${WHITELIST[*]}"; unset IFS
  echo "[INFO] [B] Build others: ${CATKIN_WHITELIST_PACKAGES//;/ }"
  catkin_make -j"$JOBS" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_TESTING=OFF
fi

echo
echo "[DONE] All builds finished."
echo "  source install_isolated/setup.bash"
