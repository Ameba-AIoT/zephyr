# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_compile_definitions(ARM_CORE_CM4=1)

zephyr_include_directories(${ZEPHYR_BASE}/drivers)

zephyr_sources(soc.c)

zephyr_include_directories(.)

zephyr_linker_sources(SECTIONS boot_section.ld)

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")

zephyr_link_libraries(
    "-Wl,-wrap,memchr"
    "-Wl,-wrap,memcmp"
    "-Wl,-wrap,memcpy"
    "-Wl,-wrap,memmove"
    "-Wl,-wrap,memset"
    "-Wl,-wrap,strcmp"
    "-Wl,-wrap,strlen"
    "-Wl,-wrap,strtoul"
    "-Wl,-wrap,strupr"
    "-Wl,-wrap,stricmp"
    "-Wl,-wrap,rand"
    "-Wl,-wrap,strchr"
    "-Wl,-wrap,strncmp"
    "-Wl,-wrap,strstr"
    "-Wl,-wrap,strncpy"
    "-Wl,-wrap,snprintf"
    "-Wl,-wrap,"
    "-Wl,-wrap,"
    "-Wl,-wrap,"
)
