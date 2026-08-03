# Copyright (c) 2025 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# This file includes extra build system logic for the MCUboot-managed app image
# that is enabled when CONFIG_BOOTLOADER_MCUBOOT=y.
# This file should be invoked by setting property SIGNING_SCRIPT of zephyr_property_target
# You can set custom output prefix by setting property ameba_output_prefix of zephyr_property_target

get_target_property(ameba_soc_name zephyr_property_target ameba_soc_name)
get_target_property(origin_secondary_image zephyr_property_target origin_secondary_image)
get_target_property(output_prefix zephyr_property_target ameba_output_prefix)

function(ameba_rsip_read_manifest out_enable out_iv)
  file(READ "${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/manifest_formatted.json" JSON_CONTENT)
  string(JSON rsip_enable GET "${JSON_CONTENT}" "image2" "rsip_enable")
  set(${out_enable} "${rsip_enable}" PARENT_SCOPE)
  if(rsip_enable)
    string(JSON image2_iv GET "${JSON_CONTENT}" "image2" "rsip_iv")
    set(${out_iv} "${image2_iv}" PARENT_SCOPE)
  endif()
endfunction()

function(zephyr_mcuboot_app_tasks_early)
  ameba_rsip_read_manifest(rsip_enable image2_iv)

  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${CMAKE_COMMAND} -E copy
        ${origin_secondary_image}
        ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app1_origin.bin
    COMMAND ${CMAKE_COMMAND} -E copy
        ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
        ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_origin.bin
  )

  if(rsip_enable)
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

function(zephyr_mcuboot_app_tasks_late)
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
  if(CONFIG_SOC_SERIES_AMEBASMART)
    # RTL8730E (AmebaSmart) MCUboot app image assembly, done here
    # (unsigned); mcuboot.cmake below signs it like every other SoC.
    #
    # App layout (imgtool-signed, --header-size 0x200):
    #   [0x200 null]  MCUboot header placeholder
    #   [ARM VT: 32B] SP=MSP_RAM_HP, Reset=KM4 DRAM entry from km4 blob,
    #                 padded from the raw 8B {MSP,Reset} pair for RSIP
    #                 MMU 32-byte alignment
    #   [km0 blob]    km0_image2_all.bin  (Realtek sub-image chain)
    #   [km4 blob]    km4_image2_all.bin  (Realtek sub-image chain)
    #   [CA32 chain]  xip_hdr(32B) + bl1_sram + bl1 + fip, each with a 32-byte
    #                 op_prepend_header sub-image header
    #
    # Must run after TF-A (consumes fip.bin) - guaranteed since this
    # SIGNING_SCRIPT is include()d at the very end of zephyr/CMakeLists.txt.

    ameba_rsip_read_manifest(rsip_enable image2_iv)
    if(rsip_enable)
      set(CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS "${CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS} --custom-tlv ${CONFIG_AMEBA_RSIP_IV_TYPE_IN_TLV} 0x${image2_iv}")
    endif()

    # ameba_layout.ld is static, so resolve its addresses at configure time.
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE}
              ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              amebasmart_boot_assets resolve-addrs
              --layout-file ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/amebasmart/ameba_layout.ld
      OUTPUT_VARIABLE ameba_layout_addrs_raw
      RESULT_VARIABLE ameba_layout_addrs_rc
    )
    if(NOT ameba_layout_addrs_rc EQUAL 0)
      message(FATAL_ERROR "amebasmart_boot_assets resolve-addrs failed (rc=${ameba_layout_addrs_rc})")
    endif()
    string(REGEX REPLACE "\n" ";" ameba_layout_addrs_lines "${ameba_layout_addrs_raw}")
    foreach(ameba_addr_line ${ameba_layout_addrs_lines})
      if(ameba_addr_line MATCHES "^([a-z0-9_]+)=(0x[0-9a-fA-F]+)$")
        set(ameba_layout_${CMAKE_MATCH_1} ${CMAKE_MATCH_2})
      endif()
    endforeach()

    # RSIP window base = XIP ORIGIN - 0x20 (Realtek sub-image header size).
    math(EXPR ameba_km0_rsip_addr  "${ameba_layout_km0_xip} - 0x20" OUTPUT_FORMAT HEXADECIMAL)
    math(EXPR ameba_km4_rsip_addr  "${ameba_layout_km4_xip} - 0x20" OUTPUT_FORMAT HEXADECIMAL)
    math(EXPR ameba_ca32_rsip_addr "${ameba_layout_xip}     - 0x20" OUTPUT_FORMAT HEXADECIMAL)

    set(ameba_work_dir ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project)
    set(ameba_tfa_img_dir ${CMAKE_BINARY_DIR}/tfa/project_ap/image)
    set(ameba_blobs_dir ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/ameba/amebasmart/bin)
    set(ameba_hal_platform_h ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/amebasmart/source/fwlib/include/hal_platform.h)

    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${CMAKE_COMMAND} -E make_directory ${ameba_work_dir}

      # CA32: pad+header each TF-A output, concat, RSIP as one chain.
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              pad --input-file ${ameba_tfa_img_dir}/bl1_sram.bin --length 32
              --output-file ${ameba_work_dir}/bl1_sram_pad.bin
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              pad --input-file ${ameba_tfa_img_dir}/bl1.bin --length 32
              --output-file ${ameba_work_dir}/bl1_pad.bin
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              pad --input-file ${ameba_tfa_img_dir}/fip.bin --length 32
              --output-file ${ameba_work_dir}/fip_pad.bin
      COMMAND ${CMAKE_COMMAND} -E touch ${ameba_work_dir}/xip_image2.bin
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              prepend_header -o ${ameba_work_dir}/xip_image2_prepend.bin
              -i ${ameba_work_dir}/xip_image2.bin --address ${ameba_layout_xip}
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              prepend_header -o ${ameba_work_dir}/bl1_sram_prepend.bin
              -i ${ameba_work_dir}/bl1_sram_pad.bin --address ${ameba_layout_bl1_sram}
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              prepend_header -o ${ameba_work_dir}/bl1_prepend.bin
              -i ${ameba_work_dir}/bl1_pad.bin --address ${ameba_layout_bl1_dram}
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              prepend_header -o ${ameba_work_dir}/fip_prepend.bin
              -i ${ameba_work_dir}/fip_pad.bin --address ${ameba_layout_fip}
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              helper merge -o ${ameba_work_dir}/ap_image_plain.bin
              -i ${ameba_work_dir}/xip_image2_prepend.bin ${ameba_work_dir}/bl1_sram_prepend.bin
                 ${ameba_work_dir}/bl1_prepend.bin ${ameba_work_dir}/fip_prepend.bin
      COMMAND ${CMAKE_COMMAND} -E chdir "${ameba_work_dir}"
              ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              rsip -i ${ameba_work_dir}/ap_image_plain.bin -o ${ameba_work_dir}/ap_image_rsip.bin
              --address ${ameba_ca32_rsip_addr} --type image2

      # KM0/KM4: RSIP the prebuilt blob directly at its own window base.
      COMMAND ${CMAKE_COMMAND} -E chdir "${ameba_work_dir}"
              ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              rsip -i ${ameba_blobs_dir}/km0_image2_all.bin -o ${ameba_work_dir}/km0_rsip.bin
              --address ${ameba_km0_rsip_addr} --type image2
      COMMAND ${CMAKE_COMMAND} -E chdir "${ameba_work_dir}"
              ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              rsip -i ${ameba_blobs_dir}/km4_image2_all.bin -o ${ameba_work_dir}/km4_rsip.bin
              --address ${ameba_km4_rsip_addr} --type image2

      # ARM VT {MSP_RAM_HP, KM4 app_start}, scanned from the plaintext KM4 blob.
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              amebasmart_boot_assets make-vt
              --km4-blob ${ameba_blobs_dir}/km4_image2_all.bin
              --km4-bd-dram-addr ${ameba_layout_km4_bd_dram}
              --hal-platform-header ${ameba_hal_platform_h}
              -o ${ameba_work_dir}/vt.bin

      # Staple [VT][km0][km4][ca32] and reserve the 0x200 mcuboot header.
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              helper merge -o ${ameba_work_dir}/app_raw.bin
              -i ${ameba_work_dir}/vt.bin ${ameba_work_dir}/km0_rsip.bin
                 ${ameba_work_dir}/km4_rsip.bin ${ameba_work_dir}/ap_image_rsip.bin
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py
              pad --input-file ${ameba_work_dir}/app_raw.bin --value 0x0 --length 0x200
              --from-head --no-align --output-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
    )
  else()
    zephyr_mcuboot_app_tasks_early()
  endif()
  include(${ZEPHYR_BASE}/cmake/mcuboot.cmake)
  zephyr_mcuboot_app_tasks_late()
else()
  # g2/dplus set this same SIGNING_SCRIPT under CONFIG_TFM_BL2 too.
  set(TFM_BINARY_DIR ${CMAKE_BINARY_DIR}/tfm)
  set(PREPROCESSED_FILE_NS "${TFM_BINARY_DIR}/bl2/ext/mcuboot/CMakeFiles/signing_layout_ns.dir/signing_layout_ns.o")
  set(TFM_MCUBOOT_DIR "${ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR}/bl2/ext/mcuboot")
  string(CONFIGURE ${CONFIG_TFM_KEY_FILE_NS} CONFIG_TFM_KEY_FILE_NS)

  # Select the Non-Secure image body (app0), mirroring the CONFIG_TFM_USE_NS_APP
  # switch in zephyr/modules/trusted-firmware-m/CMakeLists.txt:
  #  - CONFIG_TFM_USE_NS_APP=y (e.g. tfm_psa_test / tfm_regression_test, where the
  #    real NS is a TF-M ExternalProject app and the Zephyr app is only a stub):
  #    use the TF-M NS binary. It is already the raw image at its link base with no
  #    ROM_START_OFFSET reserved header, so copy it directly (no cut).
  #  - otherwise: the NS is the Zephyr app, whose image carries a ROM_START_OFFSET
  #    reserved header that must be cut off first.
  if(CONFIG_TFM_USE_NS_APP)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${CMAKE_COMMAND} -E copy
              $<TARGET_PROPERTY:tfm,TFM_NS_BIN_FILE>
              ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_cuted.bin
    )
  else()
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py cut
              --input-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.bin
              --output-file ${CMAKE_BINARY_DIR}/${ameba_soc_name}_gcc_project/app0_cuted.bin
              --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot
    )
  endif()

  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
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
