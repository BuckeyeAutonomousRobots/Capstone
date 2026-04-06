#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${CONTAINER:-pick_place_gazebo}"
BUILD_PACKAGES="robotiq_hande_description ur_hande_description ur_moveit_config servo_test_config teleop_bridge_msgs pick_place_teleop"
TELEOP_CONFIG_FILE="/home/noah/ws_moveit/src/pick_place_teleop/config/teleop.yaml"
PROCESS_PATTERN="keyboard_target_twist_teleop|target_twist_to_servo_cmd|target_twist_to_gripper_cmd|target_twist_reset_manager|run_tabletop_sim.sh|servo_gz.launch.py"

dc() {
  (cd "${ROOT_DIR}" && docker compose "$@")
}

container_running() {
  docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"
}

require_running() {
  if ! container_running; then
    echo "[error] Container ${CONTAINER} is not running."
    echo "        Start it with: ./pick_place_lifecycle.sh up_container_build"
    exit 1
  fi
}

dexec() {
  docker exec "${CONTAINER}" bash -lc "$1"
}

print_container_status() {
  docker ps -a --filter "name=^/${CONTAINER}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'
}

show_container_logs() {
  echo "[info] Recent container logs:"
  docker logs "${CONTAINER}" 2>/dev/null || true
}

workspace_ready() {
  if ! container_running; then
    return 1
  fi
  docker exec "${CONTAINER}" bash -lc "source /opt/ros/humble/setup.bash && \
    [ -f /home/noah/ws_moveit/install/setup.bash ] && \
    source /home/noah/ws_moveit/install/setup.bash && \
    ros2 pkg prefix pick_place_teleop >/dev/null 2>&1 && \
    ros2 pkg prefix servo_test_config >/dev/null 2>&1"
}

build_ws() {
  require_running
  dexec "source /opt/ros/humble/setup.bash && cd /home/noah/ws_moveit && \
    colcon build --packages-select ${BUILD_PACKAGES}"
  echo "[ok] Built workspace packages: ${BUILD_PACKAGES}"
}

ensure_ws_built() {
  if workspace_ready; then
    echo "[ok] Workspace already built."
    return 0
  fi
  echo "[info] Workspace not ready in container. Building now..."
  build_ws
}

stop_nodes() {
  if ! container_running; then
    echo "[info] ${CONTAINER} not running; skip node stop."
    return 0
  fi

  dexec "self=\$\$; \
    for p in \$(pgrep -f '${PROCESS_PATTERN}' || true); do \
      [ \"\$p\" = \"\$self\" ] && continue; \
      kill -2 \"\$p\" 2>/dev/null || true; \
    done; \
    sleep 1; \
    for p in \$(pgrep -f '${PROCESS_PATTERN}' || true); do \
      [ \"\$p\" = \"\$self\" ] && continue; \
      kill -9 \"\$p\" 2>/dev/null || true; \
    done"

  echo "[ok] Stopped simulation/control processes inside ${CONTAINER}."
}

safe_down() {
  stop_nodes
  dc down --remove-orphans || true
  echo "[ok] Compose stack is down."
}

up_container() {
  dc up -d
  sleep 1
  print_container_status
  if ! container_running; then
    echo "[error] ${CONTAINER} exited during startup."
    show_container_logs
    echo "[hint] Try rebuilding the image with: ./pick_place_lifecycle.sh up_container_build"
    return 1
  fi
}

up_container_build() {
  dc up -d --build
  sleep 1
  print_container_status
  if ! container_running; then
    echo "[error] ${CONTAINER} exited during startup even after rebuild."
    show_container_logs
    return 1
  fi
}

start_sim() {
  require_running
  ensure_ws_built
  dexec "source /opt/ros/humble/setup.bash && source /home/noah/ws_moveit/install/setup.bash && \
    self=\$\$; \
    for p in \$(pgrep -f 'run_tabletop_sim.sh' || true); do \
      [ \"\$p\" = \"\$self\" ] && continue; kill -2 \"\$p\" 2>/dev/null || true; \
    done; \
    sleep 0.4; \
    for p in \$(pgrep -f 'run_tabletop_sim.sh' || true); do \
      [ \"\$p\" = \"\$self\" ] && continue; kill -9 \"\$p\" 2>/dev/null || true; \
    done; \
    nohup env SIM_HEADLESS=${SIM_HEADLESS:-0} /home/noah/ws_moveit/simulation/launch/run_tabletop_sim.sh >/tmp/run_tabletop_sim.log 2>&1 < /dev/null &"
  echo "[ok] Started Gazebo tabletop simulation."
}

start_servo() {
  require_running
  ensure_ws_built
  dexec "source /opt/ros/humble/setup.bash && source /home/noah/ws_moveit/install/setup.bash && \
    self=\$\$; \
    for p in \$(pgrep -f 'servo_gz.launch.py' || true); do \
      [ \"\$p\" = \"\$self\" ] && continue; kill -2 \"\$p\" 2>/dev/null || true; \
    done; \
    sleep 0.4; \
    for p in \$(pgrep -f 'servo_gz.launch.py' || true); do \
      [ \"\$p\" = \"\$self\" ] && continue; kill -9 \"\$p\" 2>/dev/null || true; \
    done; \
    nohup /opt/ros/humble/bin/ros2 launch servo_test_config servo_gz.launch.py >/tmp/servo_gz.log 2>&1 < /dev/null &"
  echo "[ok] Started MoveIt Servo."
}

start_control() {
  require_running
  ensure_ws_built
  dexec "source /opt/ros/humble/setup.bash && source /home/noah/ws_moveit/install/setup.bash && \
    self=\$\$; \
    for p in \$(pgrep -f 'target_twist_to_servo_cmd|target_twist_to_gripper_cmd|target_twist_reset_manager' || true); do \
      [ \"\$p\" = \"\$self\" ] && continue; kill -2 \"\$p\" 2>/dev/null || true; \
    done; \
    sleep 0.4; \
    for p in \$(pgrep -f 'target_twist_to_servo_cmd|target_twist_to_gripper_cmd|target_twist_reset_manager' || true); do \
      [ \"\$p\" = \"\$self\" ] && continue; kill -9 \"\$p\" 2>/dev/null || true; \
    done; \
    nohup /opt/ros/humble/bin/ros2 run pick_place_teleop target_twist_to_servo_cmd --ros-args --params-file ${TELEOP_CONFIG_FILE} >/tmp/target_to_servo.log 2>&1 < /dev/null & \
    nohup /opt/ros/humble/bin/ros2 run pick_place_teleop target_twist_to_gripper_cmd --ros-args --params-file ${TELEOP_CONFIG_FILE} >/tmp/target_to_gripper.log 2>&1 < /dev/null & \
    nohup /opt/ros/humble/bin/ros2 run pick_place_teleop target_twist_reset_manager --ros-args --params-file ${TELEOP_CONFIG_FILE} >/tmp/reset_manager.log 2>&1 < /dev/null &"
  echo "[ok] Started controller bridge nodes."
}

run_keyboard() {
  require_running
  ensure_ws_built
  docker exec -it "${CONTAINER}" bash -lc "source /opt/ros/humble/setup.bash && \
    source /home/noah/ws_moveit/install/setup.bash && \
    ros2 run pick_place_teleop keyboard_target_twist_teleop --ros-args --params-file ${TELEOP_CONFIG_FILE}"
}

bringup_all() {
  up_container_build
  ensure_ws_built
  start_sim
  sleep 2
  start_servo
  sleep 2
  start_control
  status
  echo
  echo "Run interactive keyboard teleop with:"
  echo "  ./pick_place_lifecycle.sh run_keyboard"
}

status() {
  docker ps --filter "name=^/${CONTAINER}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'
  if container_running; then
    dexec "echo '--- processes'; pgrep -fa '${PROCESS_PATTERN}' || true"
    dexec "echo '--- recent logs'; \
      for f in /tmp/run_tabletop_sim.log /tmp/servo_gz.log /tmp/target_to_servo.log /tmp/target_to_gripper.log /tmp/reset_manager.log; do \
        echo '===== ' \$f; [ -f \$f ] && tail -n 12 \$f || echo missing; \
      done"
  fi
}

usage() {
  cat <<'EOF'
Usage:
  ./pick_place_lifecycle.sh safe_down
  ./pick_place_lifecycle.sh up_container
  ./pick_place_lifecycle.sh up_container_build
  ./pick_place_lifecycle.sh restart_container
  ./pick_place_lifecycle.sh build_ws
  ./pick_place_lifecycle.sh start_sim
  ./pick_place_lifecycle.sh start_servo
  ./pick_place_lifecycle.sh start_control
  ./pick_place_lifecycle.sh bringup_all
  ./pick_place_lifecycle.sh run_keyboard
  ./pick_place_lifecycle.sh stop_nodes
  ./pick_place_lifecycle.sh status
EOF
}

cmd="${1:-}"
case "${cmd}" in
  safe_down) safe_down ;;
  up_container) up_container ;;
  up_container_build) up_container_build ;;
  restart_container) safe_down; up_container ;;
  build_ws) build_ws ;;
  start_sim) start_sim ;;
  start_servo) start_servo ;;
  start_control) start_control ;;
  bringup_all) bringup_all ;;
  run_keyboard) run_keyboard ;;
  stop_nodes) stop_nodes ;;
  status) status ;;
  *) usage; exit 1 ;;
esac
