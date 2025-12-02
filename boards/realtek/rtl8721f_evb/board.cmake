# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(amebaflash "--image-dir=${ZEPHYR_BINARY_DIR}/../images" "--device=${CONFIG_SOC_SERIES}")
board_set_flasher_ifnset(amebaflash)
board_finalize_runner_args(amebaflash)

board_runner_args(jlink "--device=Cortex-M55" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
