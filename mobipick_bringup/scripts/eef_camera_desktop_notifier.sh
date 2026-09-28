#!/usr/bin/env bash
# Desktop notifications on the laptop when the real robot's eef camera goes down and when it is back.
# Follows the latched status of nodes/camera_driver_relauncher.py through a running Mobipick Labs GUI
# container that uses a remote ROS master (real robot only: sim sessions are skipped) and calls notify-send.
# Run it on the host (the GUI laptop), not in a container: the GUI blocks containers from the desktop
# notification service. Leave it running; it waits for a real-robot session and reconnects on its own.
#
#   eef_camera_desktop_notifier.sh [--any-master] [--dry] [status_topic]
#   eef_camera_desktop_notifier.sh --stdin   # test: status lines ("data: ...") from stdin, notifications printed
# Tests in the sim: --any-master also follows a container with a local master (sim), --dry prints the
# notifications instead of showing them.
TOPIC=/mobipick/eef_main_cam_driver_relauncher/status
NOTIFY=(notify-send -a "Mobipick" -i camera-web)
TEST=""
ANY_MASTER=""
for arg in "$@"; do
  case "$arg" in
    --stdin) TEST=1; NOTIFY=(echo NOTIFY) ;;
    --dry) NOTIFY=(echo NOTIFY) ;;
    --any-master) ANY_MASTER=1 ;;
    ?*) TOPIC=$arg ;;
  esac
done
state=""  # ok | down | missing

real_robot_container() {
  local c master
  for c in $(docker ps --filter name=mpcmd- --format '{{.Names}}'); do
    master=$(docker inspect "$c" --format '{{range .Config.Env}}{{println .}}{{end}}' | sed -n 's/^ROS_MASTER_URI=//p')
    [ -n "$ANY_MASTER" ] && { echo "$c"; return; }
    # the sim's master is the GUI's own roscore container (mobipick-roscore), the real robot's is remote
    case "$master" in ""|*localhost*|*127.0.0.1*|*mobipick-roscore*) ;; *) echo "$c"; return ;; esac
  done
}

follow() {
  local line status new
  while IFS= read -r line; do
    case "$line" in data:*) ;; *) continue ;; esac
    status=${line#data: }; status=${status#\"}; status=${status%\"}
    case "$status" in
      starting) continue ;;
      ok) new=ok ;;
      "device missing"*) new=missing ;;
      *) new=down ;;
    esac
    [ "$new" = "$state" ] && continue
    case "$new" in
      down) "${NOTIFY[@]}" -u critical "Arm camera down" "No images from the eef camera: $status" ;;
      missing) "${NOTIFY[@]}" -u critical "Arm camera unplugged" "USB device gone: replug the eef camera" ;;
      ok) [ -n "$state" ] && "${NOTIFY[@]}" "Arm camera back" "The eef camera sends images again" ;;
    esac
    state=$new
  done
}

if [ -n "$TEST" ]; then
  follow
  exit 0
fi
# the rostopic echo inside the container outlives the docker exec client: stop it when the notifier exits
trap '[ -n "$container" ] && docker exec "$container" pkill -f "rostopic echo $TOPIC" >/dev/null 2>&1' EXIT
trap 'exit 0' INT TERM
while true; do
  container=$(real_robot_container)
  if [ -n "$container" ]; then
    follow < <(docker exec "$container" /usr/local/bin/entrypoint_user.sh bash -lc "rostopic echo $TOPIC" 2>/dev/null)
  fi
  sleep 15
done
