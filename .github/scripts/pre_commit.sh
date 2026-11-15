#!/usr/bin/env sh

cmake --preset $1
cmake --build --preset $1 --target autogen_platform
