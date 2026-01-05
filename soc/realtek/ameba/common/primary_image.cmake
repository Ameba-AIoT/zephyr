# Copyright (c) 2025 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# This file includes extra build system logic for primary image that is enabled when
# CONFIG_BOOTLOADER_MCUBOOT=y.
# This file should be invoked by setting property SIGNING_SCRIPT of zephyr_property_target
# You can set custom output prefix by setting property ameba_output_prefix of zephyr_property_target

function(zephyr_primary_image_tasks)
  set(output ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME})

  get_target_property(output_prefix zephyr_property_target ameba_output_prefix)
  if("${output_prefix}" STREQUAL "")
    set(output_prefix zephyr)
  endif()
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${CMAKE_COMMAND} -E copy
            ${output}.signed.bin
            ${CMAKE_BINARY_DIR}/images/${output_prefix}.signed.bin)

  if (CONFIG_MCUBOOT_ENCRYPTION_KEY_FILE)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${CMAKE_COMMAND} -E copy
              ${output}.signed.encrypted.bin
              ${CMAKE_BINARY_DIR}/images/${output_prefix}.signed.encrypted.bin)
  endif()
endfunction()

include(${ZEPHYR_BASE}/cmake/mcuboot.cmake)
zephyr_primary_image_tasks()
