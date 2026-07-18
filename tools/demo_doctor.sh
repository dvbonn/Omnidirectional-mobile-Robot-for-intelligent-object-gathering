#!/bin/bash
# ============================================================================
# demo_doctor.sh - RESCUE helper when the T10 demo (detect -> drive to object) fails.
# Bundles the common issues on the Jetson (ROS2 Foxy + Astra USB2 + hotspot):
#   - Orphan node holding the Astra -> camera_manager: "oniDeviceOpen ... rc=4118"
#   - rosbridge port conflict       -> "Address already in use [Errno 98]" (9090)
#   - stale ros2 daemon / topics not seeing each other
#   - unknown hotspot IP for opening Foxglove
#
# USAGE:
#   tools/demo_doctor.sh            # = fix: clean up + restart daemon + check + print run commands
#   tools/demo_doctor.sh clean      # only kill ROS orphans + free the Astra + release port 9090
#   tools/demo_doctor.sh check      # only check Astra/USB, ROS nodes, port 9090
#   tools/demo_doctor.sh net        # only check the hotspot + print the Foxglove URL
#   tools/demo_doctor.sh launch     # clean up THEN run collect (args after)
#                                   #   e.g.: tools/demo_doctor.sh launch odom:=fake nav:=false target_class:=bottle
#
# NOTE: Foxy does NOT support `ros2 topic echo --once` (Galactic+ only). And a
#    `ros2 topic echo` running OUTSIDE usually cannot join the DDS of the node group
#    spawned by `ros2 launch` - watch the launch LOG or Foxglove, do not trust an external echo.
# ============================================================================
set -u

REPO_ROOT="${ROBOT_REPO_ROOT:-/home/asiclab/Robot_collecting_VLM_Model}"
ROS_WS="${ROS_WS:-$REPO_ROOT/ros2_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WLAN_IF="${WLAN_IF:-wlan0}"
AP_IP="${AP_IP:-192.168.137.1}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-9090}"

# Process patterns of the demo stack (collect/bringup/astra_scan + all child nodes)
ROS_PAT='camera_manager|astra_node|astra_color_node|orchestrator_node|yolo_node|async_slam_toolbox|depthimage_to_laserscan|rosbridge_websocket|rosapi_node|fake_odom|wifi_node|robot_state_publisher|nav2_|planner_server|controller_server|bt_navigator|lifecycle_manager|recoveries_server|collect.launch|bringup.launch|astra_scan.launch'

c_ok(){ printf '  \033[32mOK\033[0m %s\n' "$*"; }
c_no(){ printf '  \033[31mX\033[0m %s\n' "$*"; }
c_hd(){ printf '\n\033[1m== %s ==\033[0m\n' "$*"; }

do_clean() {
  c_hd "CLEAN UP ROS PROCESSES (free the Astra + port $FOXGLOVE_PORT)"
  # 1) Gracefully stop the parent launches first (SIGINT propagates to children -> clean camera close)
  local parents
  parents=$(pgrep -f 'ros2 launch .*\.launch\.py' || true)
  if [ -n "$parents" ]; then
    echo "  SIGINT parent launch: $parents"
    kill -2 $parents 2>/dev/null || true
  fi
  # 2) Wait up to 10s for a graceful shutdown
  for i in $(seq 1 10); do
    [ -z "$(pgrep -f "$ROS_PAT" | grep -v $$ || true)" ] && break
    sleep 1
  done
  # 3) Force-kill anything left
  local left
  left=$(pgrep -f "$ROS_PAT" | grep -v $$ || true)
  if [ -n "$left" ]; then
    echo "  SIGKILL remaining: $(echo $left | tr '\n' ' ')"
    kill -9 $left 2>/dev/null || true
    sleep 1
  fi
  # 4) Release the Foxglove port if anyone still holds it
  if command -v fuser >/dev/null 2>&1; then
    fuser -k "${FOXGLOVE_PORT}/tcp" 2>/dev/null && echo "  released port $FOXGLOVE_PORT" || true
  fi
  # 5) Restart the ros2 daemon (clear ghost discovery)
  ( source "$ROS_SETUP" 2>/dev/null; ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1 )
  local rem
  rem=$(pgrep -f "$ROS_PAT" | grep -v $$ | wc -l)
  [ "$rem" -eq 0 ] && c_ok "no ROS nodes left (Astra + port freed)" \
                   || c_no "still $rem processes - run 'clean' again or check manually"
}

do_check() {
  c_hd "CHECK HARDWARE / PROCESSES"
  # Astra USB (Orbbec 2bc5)
  if lsusb 2>/dev/null | grep -qiE '2bc5|orbbec'; then
    c_ok "Astra USB: $(lsusb | grep -iE '2bc5|orbbec' | sed 's/^/       /')"
  else
    c_no "Astra NOT seen over USB -> replug the cable / try another USB port / check power"
  fi
  # ROS nodes running?
  local n
  n=$(pgrep -f "$ROS_PAT" | grep -v $$ | wc -l)
  [ "$n" -eq 0 ] && c_ok "no ROS nodes running (ready for a new launch)" \
                 || c_no "$n ROS nodes running - a new launch would clash over Astra/port, run 'clean' first"
  # Foxglove port
  if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -q ":${FOXGLOVE_PORT}\b"; then
    c_no "port $FOXGLOVE_PORT is IN USE - 'clean' to release"
  else
    c_ok "port $FOXGLOVE_PORT free"
  fi
}

do_net() {
  c_hd "HOTSPOT / NETWORK (Foxglove)"
  # wlan0 up + IP
  local ip
  ip=$(ip -4 -br addr show "$WLAN_IF" 2>/dev/null | awk '{print $3}')
  if [ -n "$ip" ]; then
    c_ok "$WLAN_IF UP, IP = $ip"
  else
    c_no "$WLAN_IF has no IP / is down -> start the hotspot: sudo $REPO_ROOT/tools/setup_hostapd.sh"
    ip="$AP_IP"
  fi
  # hostapd + dnsmasq
  pgrep -x hostapd >/dev/null 2>&1 && c_ok "hostapd broadcasting (SSID: Jetson)" \
                                   || c_no "hostapd NOT running -> sudo $REPO_ROOT/tools/setup_hostapd.sh"
  pgrep -x dnsmasq >/dev/null 2>&1 && c_ok "dnsmasq (DHCP) running" \
                                   || c_no "dnsmasq NOT running (clients will not get an IP)"
  # Connected clients (needs privilege - try without sudo first)
  local sta
  sta=$(iw dev "$WLAN_IF" station dump 2>/dev/null | grep -c Station); sta="${sta:-0}"
  echo "  Devices connected to the AP: ${sta}"
  printf '\n  \033[1mFoxglove:\033[0m open the app -> Open connection -> \033[36mws://%s:%s\033[0m\n' "${ip%/*}" "$FOXGLOVE_PORT"
}

print_launch_hint() {
  c_hd "RE-RUN COMMANDS (copy the whole block)"
  cat <<EOF
  cd $ROS_WS && source $ROS_SETUP && source install/setup.bash

  # Stage A - vision+localization only (NO base needed, robot does not move):
  ros2 launch robot_bringup collect.launch.py odom:=fake nav:=false target_class:=bottle

  # Stage B - detect -> drive to the object (real base, place a bottle >0.8m away):
  ros2 launch robot_bringup collect.launch.py target_class:=bottle

  # Watch progress (TRUSTED channel, instead of an external 'ros2 topic echo'):
  #   read the orchestrator log: "Object detected -> LOCATE", "Object @map (x,y) -> goal (x,y)"
EOF
}

CMD="${1:-fix}"
case "$CMD" in
  clean)  do_clean ;;
  check)  do_check ;;
  net)    do_net ;;
  fix)    do_clean; do_check; do_net; print_launch_hint ;;
  launch)
    shift
    do_clean
    c_hd "RUN collect.launch.py $*"
    cd "$ROS_WS" || exit 1
    # shellcheck disable=SC1090
    source "$ROS_SETUP" 2>/dev/null; source install/setup.bash 2>/dev/null
    exec ros2 launch robot_bringup collect.launch.py "$@"
    ;;
  *) echo "Usage: $0 [fix|clean|check|net|launch ...]"; exit 2 ;;
esac
