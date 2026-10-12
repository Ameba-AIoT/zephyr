# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(ameba_at_shell)
zephyr_library_sources(ameba_at_shell.c)
zephyr_linker_sources(SECTIONS ameba_at_cmd.ld)
