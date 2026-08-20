#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
env_file="${repo_root}/.env"

if [[ ! -f "${env_file}" ]]; then
  echo "Missing ${env_file}. Copy .env.example to .env and set external paths." >&2
  exit 1
fi

set -a
# shellcheck disable=SC1090
source "${env_file}"
set +a

base_image="${AUTOWARE_BASE_IMAGE:-}"
vtd_image="${VTD_IMAGE:-selfcar-2026-vtd:local}"
vtd_install_dir="${VTD_INSTALL_DIR:-}"

if [[ -z "${base_image}" || "${base_image}" != *@sha256:* ]]; then
  echo "AUTOWARE_BASE_IMAGE must include an immutable @sha256 digest." >&2
  exit 1
fi
if [[ -z "${vtd_install_dir}" || ! -d "${vtd_install_dir}" ]]; then
  echo "VTD_INSTALL_DIR is missing or does not exist: ${vtd_install_dir:-<unset>}" >&2
  exit 1
fi
vtd_header="${vtd_install_dir}/Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit/viRDBIcd.h"
if [[ ! -f "${vtd_header}" ]]; then
  echo "VTD RDB header was not found: ${vtd_header}" >&2
  exit 1
fi
vtd_common_header="${vtd_install_dir}/Develop/Communication/Common/viRDBTypes.h"
if [[ ! -f "${vtd_common_header}" ]]; then
  echo "VTD common header was not found: ${vtd_common_header}" >&2
  exit 1
fi

if ! docker buildx version >/dev/null 2>&1; then
  echo "Docker Buildx is required to provide the read-only VTD named context." >&2
  exit 1
fi

# The bridge only needs these two public headers at compile time. Stage that
# minimal, read-only build context so an installed VTD distribution's maps,
# binaries, and other large runtime assets are never sent to the Docker daemon.
vtd_context="$(mktemp -d /tmp/selfcar-vtd-context.XXXXXX)"
cleanup() {
  rm -rf -- "${vtd_context}"
}
trap cleanup EXIT
mkdir -p "${vtd_context}/Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit" \
  "${vtd_context}/Develop/Communication/Common"
cp -p -- "${vtd_header}" \
  "${vtd_context}/Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit/viRDBIcd.h"
cp -p -- "${vtd_common_header}" \
  "${vtd_context}/Develop/Communication/Common/viRDBTypes.h"

docker buildx build \
  --file "${repo_root}/docker/vtd/Dockerfile" \
  --build-arg "AUTOWARE_BASE_IMAGE=${base_image}" \
  --build-context "vtd=${vtd_context}" \
  --tag "${vtd_image}" \
  --load \
  "${repo_root}"
