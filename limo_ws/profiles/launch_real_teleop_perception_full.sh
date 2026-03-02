#!/usr/bin/env bash
set -euo pipefail

echo "[deprecated] Use 'bash profiles/launch_real_base_teleop.sh' instead."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
exec bash "$SCRIPT_DIR/launch_real_base_teleop.sh" "$@"
