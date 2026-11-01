#!/usr/bin/env sh
set -euo pipefail

cmake --preset $1
cmake --build --preset $1 --target autogen_platform
