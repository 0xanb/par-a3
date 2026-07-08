#!/usr/bin/env bash
# PAR-A3 dev entry point
# Usage:
#   scripts/dev.sh up       # start colima + open dev container shell
#   scripts/dev.sh down     # stop the dev container (keeps VM)
#   scripts/dev.sh stop     # stop colima VM entirely
#   scripts/dev.sh shell    # open another shell in the running container
#   scripts/dev.sh build    # colcon build inside the container
#   scripts/dev.sh test     # colcon test inside the container
#   scripts/dev.sh code     # open VS Code attached to the container
#   scripts/dev.sh status   # what is running, what is not

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=par-a3-dev
CONTAINER=par-a3-dev

colima_up() {
  if ! colima status >/dev/null 2>&1; then
    echo "[dev] starting colima…"
    colima start --cpu 4 --memory 8 --disk 60 --vm-type vz --mount-type virtiofs
  else
    echo "[dev] colima already up"
  fi
}

build_image() {
  echo "[dev] building image ${IMAGE}"
  docker build -t "${IMAGE}" -f "${ROOT}/.devcontainer/Dockerfile" "${ROOT}/.devcontainer"
}

ensure_container() {
  if ! docker inspect "${CONTAINER}" >/dev/null 2>&1; then
    docker image inspect "${IMAGE}" >/dev/null 2>&1 || build_image
    echo "[dev] creating container ${CONTAINER}"
    docker run -d --name "${CONTAINER}" \
      --hostname par-dev \
      --network host \
      -v "${ROOT}/workspace:/workspace" \
      -v "${ROOT}/docs:/docs" \
      -v "${ROOT}/scripts:/scripts" \
      -e ROS_DOMAIN_ID=0 \
      "${IMAGE}" sleep infinity >/dev/null
  fi
  docker start "${CONTAINER}" >/dev/null 2>&1 || true
}

cmd_up() {
  colima_up
  ensure_container
  docker exec -it "${CONTAINER}" bash -lc "cd /workspace && exec bash"
}

cmd_shell() {
  docker exec -it "${CONTAINER}" bash -lc "cd /workspace && exec bash"
}

cmd_build() {
  docker exec -it "${CONTAINER}" bash -lc "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install"
}

cmd_test() {
  docker exec -it "${CONTAINER}" bash -lc "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon test --event-handlers console_direct+ && colcon test-result --verbose"
}

cmd_down() { docker stop "${CONTAINER}" >/dev/null 2>&1 || true; echo "[dev] container stopped"; }
cmd_stop() { cmd_down; colima stop 2>&1 | sed 's/^/[colima] /' ; }

cmd_code() {
  colima_up; ensure_container
  code --folder-uri "vscode-remote://attached-container+$(printf '%s' "${CONTAINER}" | xxd -p | tr -d '\n')/workspace"
}

cmd_status() {
  echo "[colima]"; colima status 2>&1 || true
  echo; echo "[container]"; docker ps -a --filter "name=${CONTAINER}" 2>&1 || echo "docker not reachable"
}

case "${1:-up}" in
  up) cmd_up ;;
  down) cmd_down ;;
  stop) cmd_stop ;;
  shell) cmd_shell ;;
  build) cmd_build ;;
  test) cmd_test ;;
  code) cmd_code ;;
  status) cmd_status ;;
  *) echo "unknown: $1"; exit 1 ;;
esac
