#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
vtd_dir="${VTD_ROOT:-/home/a/VIRES/VTD.2025.2}"
autoware_image="${AUTOWARE_IMAGE:-ghcr.io/autowarefoundation/autoware:universe-devel-cuda-jazzy}"
cyclonedds_config="${CYCLONEDDS_CONFIG:-/home/a/autoware/docker/files/cyclonedds.xml}"

if [[ ! -f "${vtd_dir}/Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit/viRDBIcd.h" ]]; then
  echo "VTD_ROOT is invalid: ${vtd_dir}" >&2
  exit 1
fi
if [[ ! -f "${cyclonedds_config}" ]]; then
  echo "CycloneDDS config was not found: ${cyclonedds_config}" >&2
  exit 1
fi

docker run --rm --network host --ipc host --entrypoint /bin/bash \
  --user "$(id -u):$(id -g)" \
  --volume "${workspace_dir}:/work/vtd_ros2_bridge_ws" \
  --volume "${vtd_dir}:/opt/vtd:ro" \
  --volume "${cyclonedds_config}:/home/aw/cyclonedds.xml:ro" \
  --workdir /work/vtd_ros2_bridge_ws \
  --env VTD_ROOT=/opt/vtd \
  "${autoware_image}" \
  -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/autoware/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    source install/setup.bash
    exec ros2 launch vtd_ros2_bridge vtd_bridge.launch.py "$@"
  ' bash "$@"
