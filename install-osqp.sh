#!/usr/bin/env bash
set -euo pipefail

script_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
osqp_version="v1.0.0"
osqp_install_prefix="${script_directory}/.deps/osqp"
osqp_config_file="${osqp_install_prefix}/lib/cmake/osqp/osqp-config.cmake"

if [[ -f "${osqp_config_file}" ]]; then
  echo "OSQP ${osqp_version} is already installed in ${osqp_install_prefix}"
  exit 0
fi

osqp_temporary_root="$(mktemp -d)"
cleanup_osqp_source() {
  rm -rf -- "${osqp_temporary_root}"
}
trap cleanup_osqp_source EXIT

osqp_source_directory="${osqp_temporary_root}/osqp"
osqp_build_directory="${osqp_source_directory}/build"
git clone --depth 1 --branch "${osqp_version}" --recurse-submodules \
  --shallow-submodules \
  https://github.com/osqp/osqp.git "${osqp_source_directory}"
cmake -S "${osqp_source_directory}" -B "${osqp_build_directory}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${osqp_install_prefix}" \
  -DOSQP_BUILD_UNITTESTS=OFF
cmake --build "${osqp_build_directory}" -- -j2
cmake --build "${osqp_build_directory}" --target install

echo "Installed OSQP ${osqp_version} in ${osqp_install_prefix}"
