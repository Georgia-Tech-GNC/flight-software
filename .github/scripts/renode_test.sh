#!/usr/bin/env sh

curl -O https://builds.renode.io/renode-1.16.1+20260904git63d4e2dd5.linux-portable.tar.gz
tar -xzf renode-1.16.1+20260904git63d4e2dd5.linux-portable.tar.gz

renode_1.16.1+20260904git63d4e2dd5-portable/renode-test renode/tests/test.robot \
    --variable VARIABLE_FILE:targets/Nucleo-h723zg/robot_config.yaml \
    --variable FIRMWARE_ELF:build/ncl-h723-dev/flight-software.elf \
    --variable PROJECT_ROOT:$PWD \
