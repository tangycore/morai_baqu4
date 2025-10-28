# ~/baqu4_morai_ws/run.sh
#!/usr/bin/env bash
set -euo pipefail

# 파이썬/ROS 콘솔 출력 지연 방지
export PYTHONUNBUFFERED=1
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1

PIDS=()
add_pid(){ PIDS+=("$1"); }
cleanup(){
  for pid in "${PIDS[@]:-}"; do kill "$pid" >/dev/null 2>&1 || true; done
  sleep 1
  for pid in "${PIDS[@]:-}"; do kill -9 "$pid" >/dev/null 2>&1 || true; done
}
trap cleanup INT TERM EXIT

launch_bg(){
  local pkg="$1"; local file="$2"
  roslaunch --wait --screen "$pkg" "$file" &
  add_pid $!
  sleep 1
}

# 여기부터 원하는 런치만 자동 기동
launch_bg baqu4_tf baqu4_tf.launch
launch_bg baqu4_udp final_mission.launch
launch_bg control control.launch
launch_bg planning_pkg planning.launch
launch_bg lidar_clustering lidar_clustering_tracking.launch

# rosbridge는 별도로:
# roslaunch --screen rosbridge_server rosbridge_websocket.launch

wait
