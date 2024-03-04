# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

set(HIFI_INC_DIR ${XTENSA_SYSTEM}/../xtensa-elf/arch/include/xtensa)
include_directories(BEFORE ${HIFI_INC_DIR})


target_sources(app PRIVATE src/main.c)
