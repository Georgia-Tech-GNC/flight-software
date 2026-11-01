#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${IN_NIX_SHELL:-}" ]]; then
    exec .github/scripts/pre_commit.sh "$@"
else
    exec nix develop --command .github/scripts/pre_commit.sh "$@"
fi
