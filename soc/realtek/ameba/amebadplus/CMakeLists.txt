# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_compile_definitions(ARM_CORE_CM4=1)

zephyr_include_directories(${ZEPHYR_BASE}/drivers)

zephyr_sources(soc.c)

zephyr_include_directories(.)

zephyr_linker_sources(SECTIONS boot_section.ld)

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
