# Copyright (c) 2025 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# This file includes extra build system logic for primary image that is enabled when
# CONFIG_BOOTLOADER_MCUBOOT=y.
# This file should be invoked by setting property SIGNING_SCRIPT of zephyr_property_target
# You can set custom output prefix by setting property ameba_output_prefix of zephyr_property_target

function(zephyr_primary_image_tasks_early)
  get_target_property(ameba_soc_name zephyr_property_target ameba_soc_name)
  get_target_property(ameba_image_name zephyr_property_target ameba_image_name)
  file(READ "${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/manifest_formatted.json" JSON_CONTENT)
  string(JSON rsip_enable GET "${JSON_CONTENT}" "image2" "rsip_enable")
  if(rsip_enable)
    string(JSON image2_iv GET "${JSON_CONTENT}" "image2" "rsip_iv")
    dt_prop(primary_logic_addr PATH "/zephyr,user" PROPERTY "primary-logic-addr")
    math(EXPR address "${primary_logic_addr} + ${CONFIG_ROM_START_OFFSET}")

    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND
        ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py cut
          --input-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
          --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/${ameba_image_name}/${KERNEL_NAME}_cuted.bin
          --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot
      COMMAND
        ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py pad
          --input-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/${ameba_image_name}/${KERNEL_NAME}_cuted.bin
          --length 32 #NOTE: rsip require 32byte alignment
      COMMAND
        ${CMAKE_COMMAND} -E chdir "${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/${ameba_image_name}/"
        ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py rsip
          --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/${ameba_image_name}/${KERNEL_NAME}_rsip_raw.bin
          --input-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/${ameba_image_name}/${KERNEL_NAME}_cuted.bin
          #WARNING: Pay attention to the offset here, which MUST be consistent with MMU config based on the real code addr
          --address ${address}
          --type image2
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py pad
          --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/${ameba_image_name}/${KERNEL_NAME}_rsip_with_head.bin
          --input-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/${ameba_image_name}/${KERNEL_NAME}_rsip_raw.bin
          --value 0x0
          --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot
          --from-head
          --no-align
      COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/${ameba_image_name}/${KERNEL_NAME}_rsip_with_head.bin
          ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
    )
    #REVIEW: Better way to set CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS with CONFIG_AMEBA_RSIP_IV_TYPE_IN_TLV?
    set(CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS "${CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS} --custom-tlv ${CONFIG_AMEBA_RSIP_IV_TYPE_IN_TLV} 0x${image2_iv}" PARENT_SCOPE)
  endif()
endfunction()

function(zephyr_primary_image_tasks_late)
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

zephyr_primary_image_tasks_early()
include(${ZEPHYR_BASE}/cmake/mcuboot.cmake)
zephyr_primary_image_tasks_late()
