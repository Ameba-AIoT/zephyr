# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_sources(ameba_irq.c)
if(CONFIG_CPU_CORTEX_M)
zephyr_sources_ifdef(CONFIG_AMEBA_PSRAM ameba_psram.c)
# reset_ameba_shell.c registers a `reboot uartburn` SHELL command; excluded for
# KM4 MCUboot because CONFIG_SHELL is not enabled in the bootloader domain.
if(NOT CONFIG_MCUBOOT)
  zephyr_sources(reset_ameba_shell.c)
endif()
endif()
