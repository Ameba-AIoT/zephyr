# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

dt_chosen(SHELLUART PROPERTY zephyr,shell-uart)
if(SHELLUART)
  dt_prop(SHELLUART_BAUDRATE PATH ${SHELLUART} PROPERTY current-speed)
  board_runner_args(amebaflash "--baudrate=${SHELLUART_BAUDRATE}")
endif()

board_runner_args(amebaflash "--image-dir=${ZEPHYR_BINARY_DIR}/../images" "--device=${CONFIG_SOC_SERIES}")
board_set_flasher_ifnset(amebaflash)
board_finalize_runner_args(amebaflash)

board_runner_args(jlink "--device=Cortex-M55" "--speed=4000")
board_runner_args(jlink "--tool-opt=-scriptfile ${CMAKE_CURRENT_LIST_DIR}/support/AP2_AP.JLinkScript")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
