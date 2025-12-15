# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=Cortex-M33" "--speed=4000")
board_runner_args(jlink "--tool-opt=-scriptfile ${CMAKE_CURRENT_LIST_DIR}/support/AP2_KM4.JLinkScript")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
