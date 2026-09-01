#!/usr/bin/env sh

cmake --preset $1 -DCMAKE_C_FLAGS="-Werror" -DCMAKE_CXX_FLAGS="-Werror"
cmake --build --preset $1

cppcheck \
        --project=build/$1/compile_commands.json \
        --enable=all \
        --suppress=missingIncludeSystem \
        --suppress=missingInclude \
        --suppress=unusedFunction \
        --suppress=unmatchedSuppression \
        --suppress=checkersReport \
        --error-exitcode=1 \
        -i targets -i lib/FreeRTOS-Kernel
