# Copyright (c) 2025 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# This file includes extra build system logic for primary image that is enabled when
# CONFIG_BOOTLOADER_MCUBOOT=y.
# This file should be invoked by setting property SIGNING_SCRIPT of zephyr_property_target
# You can set custom output prefix by setting property ameba_output_prefix of zephyr_property_target

get_target_property(ameba_soc_name zephyr_property_target ameba_soc_name)
get_target_property(origin_secondary_image zephyr_property_target origin_secondary_image)
get_target_property(output_prefix zephyr_property_target ameba_output_prefix)

function(zephyr_primary_image_tasks_early)
  file(READ "${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/manifest_formatted.json" JSON_CONTENT)
  string(JSON rsip_enable GET "${JSON_CONTENT}" "image2" "rsip_enable")


  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${CMAKE_COMMAND} -E copy
        ${origin_secondary_image}
        ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app1_origin.bin
    COMMAND ${CMAKE_COMMAND} -E copy
        ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
        ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_origin.bin
  )

  if(rsip_enable)
    string(JSON image2_iv GET "${JSON_CONTENT}" "image2" "rsip_iv")
    dt_prop(primary_logic_addr PATH "/zephyr,user" PROPERTY "primary-logic-addr")
    dt_prop(secondary_logic_addr PATH "/zephyr,user" PROPERTY "secondary-logic-addr")
    math(EXPR address "${primary_logic_addr} + ${CONFIG_ROM_START_OFFSET}")

    #REVIEW: Better way to set CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS with CONFIG_AMEBA_RSIP_IV_TYPE_IN_TLV?
    set(CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS "${CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS} --custom-tlv ${CONFIG_AMEBA_RSIP_IV_TYPE_IN_TLV} 0x${image2_iv}" PARENT_SCOPE)

    # app img0: primary image
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND
        ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py cut
          --input-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
          --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_cuted.bin
          --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot
      COMMAND
        ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py pad
          --input-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_cuted.bin
          --length 32 #NOTE: rsip require 32byte alignment
      COMMAND
        ${CMAKE_COMMAND} -E chdir "${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/"
        ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py rsip
          --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_rsip_raw.bin
          --input-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_cuted.bin
          #WARNING: Pay attention to the offset here, which MUST be consistent with MMU config based on the real code addr
          --address ${address}
          --type image2
    )

    # app img1: secondary image
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${CMAKE_COMMAND} -E copy
          ${origin_secondary_image}
          ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app1_cuted.bin
      COMMAND
        ${CMAKE_COMMAND} -E chdir "${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/"
        ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py rsip
          --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app1_rsip_raw.bin
          --input-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app1_cuted.bin
          #WARNING: Pay attention to the offset here, which MUST be consistent with MMU config based on the real code addr
          --address ${secondary_logic_addr}
          --type image2
    )

    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      # merge two app images
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py helper merge
              --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app_rsip_raw.bin
              --input-file
                ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app1_rsip_raw.bin
                ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_rsip_raw.bin

      # pad reserved mcuboot header to front
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py pad
              --input-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app_rsip_raw.bin
              --value 0x0
              --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot
              --from-head
              --no-align
              --output-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
    )
  else() # !rsip_enable
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND
        ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py cut
          --input-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
          --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_cuted.bin
          --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot

      # merge two app images
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py helper merge
              --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app_raw.bin
              --input-file
                ${origin_secondary_image}
                ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_cuted.bin

      # pad reserved mcuboot header to front
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py pad
              --input-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app_raw.bin
              --value 0x0
              --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot
              --from-head
              --no-align
              --output-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
    )
  endif()
endfunction()

function(zephyr_primary_image_tasks_late)
  set(output ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME})

  if("${output_prefix}" STREQUAL "")
    set(output_prefix zephyr)
  endif()
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${CMAKE_COMMAND} -E copy
            ${output}.signed.bin
            ${CMAKE_BINARY_DIR}/images/${output_prefix}.bin)

  if (CONFIG_MCUBOOT_ENCRYPTION_KEY_FILE)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${CMAKE_COMMAND} -E copy
              ${output}.signed.encrypted.bin
              ${CMAKE_BINARY_DIR}/images/${output_prefix}.bin)
  endif()
endfunction()

if (CONFIG_BOOTLOADER_MCUBOOT)
  zephyr_primary_image_tasks_early()
  include(${ZEPHYR_BASE}/cmake/mcuboot.cmake)
  zephyr_primary_image_tasks_late()
else()
  set(TFM_BINARY_DIR ${CMAKE_BINARY_DIR}/tfm)
  set(PREPROCESSED_FILE_NS "${TFM_BINARY_DIR}/bl2/ext/mcuboot/CMakeFiles/signing_layout_ns.dir/signing_layout_ns.o")
  set(TFM_MCUBOOT_DIR "${ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR}/bl2/ext/mcuboot")
  string(CONFIGURE ${CONFIG_TFM_KEY_FILE_NS} CONFIG_TFM_KEY_FILE_NS)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py cut
            --input-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
            --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_cuted.bin
            --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot
    COMMAND ${CMAKE_COMMAND} -E cat ${origin_secondary_image} ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_cuted.bin
                > ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app.bin
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py pad
            --input-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app.bin
            --value 0x0
            --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot
            --from-head
            --no-align
            --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app_pad.bin
    COMMAND ${PYTHON_EXECUTABLE} ${IMGTOOL} sign
          --version ${CONFIG_TFM_IMAGE_VERSION_NS}
          --header-size ${CONFIG_ROM_START_OFFSET}
          --slot-size 0x80000
          --align 1
          --public-key-format full
          -k ${CONFIG_TFM_KEY_FILE_NS}
          -v ${CONFIG_TFM_IMAGE_VERSION_NS}
          -s ${CONFIG_TFM_IMAGE_SECURITY_COUNTER}
          --boot-record NSPE #REVIEW: Need check option in tfm
          ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app_pad.bin
          ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app_signed.bin
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app_signed.bin
            ${CMAKE_BINARY_DIR}/images/${output_prefix}.bin
  )
endif()

set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts ${CMAKE_BINARY_DIR}/images/${output_prefix}.bin)
