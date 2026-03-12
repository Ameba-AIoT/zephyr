# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

add_subdirectory(os_wrapper)
zephyr_sources_ifdef(CONFIG_AMEBA_PSRAM ameba_psram.c)
zephyr_sources(reset_ameba_shell.c)
zephyr_sources(ameba_irq.c)
