#!/usr/bin/env bash
set -euo pipefail

brew install boost cmake git

script_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bash "${script_directory}/install-osqp.sh"
