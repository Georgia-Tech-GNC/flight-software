#!/usr/bin/env sh

renode-test renode/tests/test.robot \
    --variable VARIABLE_FILE:targets/Nucleo-h723zg/robot_config.yaml \
    --variable FIRMWARE_ELF:build/ncl-h723-dev/flight-software.elf \
    --variable PROJECT_ROOT:$PWD \
