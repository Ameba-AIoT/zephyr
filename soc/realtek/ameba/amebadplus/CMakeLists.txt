# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_include_directories(.)

zephyr_sources(soc.c)

zephyr_linker_sources(SECTIONS boot_section.ld)
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

zephyr_link_libraries(
    -T${CMAKE_CURRENT_SOURCE_DIR}/ameba_rom_symbol_acut_s.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/ameba_rom_symbol_stdlib.ld
  )
