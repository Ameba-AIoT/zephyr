# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
#
# Board-level sysbuild hook for RTL8730E EVB.
# Included by images/boards/CMakeLists.txt AFTER ExternalZephyrProject_Add()
# for all images but BEFORE ExternalZephyrProject_Cmake() is called.
#
# This hook normalises both forms to the correct per-domain boards.
if(TARGET ${DEFAULT_IMAGE})
  # Primary app domain always uses the CA32 base board (no //mcuboot qualifier).
  set_target_properties(${DEFAULT_IMAGE} PROPERTIES BOARD "rtl8730e_evb")
endif()
if(TARGET mcuboot)
  # MCUboot domain always uses the KM4 //mcuboot variant.
  set_target_properties(mcuboot PROPERTIES BOARD "rtl8730e_evb//mcuboot")
endif()
