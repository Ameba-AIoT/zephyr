# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_include_directories(.)

if(CONFIG_MCUBOOT)
  zephyr_linker_sources(ROM_START SORT_KEY 0x0 loader_section.ld)
  zephyr_sources(mcuboot_action_hook.c)
  zephyr_library_sources_ifdef(CONFIG_MCUBOOT_HW_DOWNGRADE_PREVENTION mcuboot_security_counters.c)
else()
  zephyr_sources(soc.c)
endif()

zephyr_linker_sources(SECTIONS boot_section.ld)
zephyr_linker_sources(SECTIONS psram_section.ld)
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

set(PYTHON_SCRIPT_ARGS
    --soc ${CONFIG_SOC_SERIES}
    --bin-file ${ZEPHYR_BINARY_DIR}/${KERNEL_BIN_NAME}
    --out-dir ${CMAKE_BINARY_DIR}
    --module-dir ${ZEPHYR_HAL_REALTEK_MODULE_DIR}
)

if(CONFIG_BT_COEXIST)
    list(APPEND PYTHON_SCRIPT_ARGS --bt-coexist)
endif()

#  west sign

file(GLOB BIN_FILES "${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/${CONFIG_SOC_SERIES}/bin/*.bin")

if(BIN_FILES)
  if (CONFIG_BOOTLOADER_MCUBOOT)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project/km4tz)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project/km4ns)
    file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/amebaG2/manifest.json5 DESTINATION ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project/)
    file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/amebaG2/ameba_layout.ld DESTINATION ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project/)
    execute_process(COMMAND_ERROR_IS_FATAL ANY
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py helper manifest-fmt
              --output-file ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project/manifest_formatted.json
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project
    )
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${CMAKE_NM} ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} | sort > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_OBJDUMP} -d ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.asm
      COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/images"
    )
    set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts ${KERNEL_NAME}.raw.map ${KERNEL_NAME}.asm)

    # post process primary image
    #NOTE: Because extra_post_build_commands added in soc is executed before that in ${ZEPHYR_BASE}/cmake/mcuboot.cmake,
    #      so mcuboot.cmake is used to wrapper ${ZEPHYR_BASE}/cmake/mcuboot.cmake with some extra extra_post_build_commands
    #      that needed to be executed exactly after ${ZEPHYR_BASE}/cmake/mcuboot.cmake
    set_target_properties(zephyr_property_target PROPERTIES SIGNING_SCRIPT
      ${CMAKE_CURRENT_LIST_DIR}/../common/primary_image.cmake)
    set_target_properties(zephyr_property_target PROPERTIES ameba_output_prefix amebagreen2_km4tz)
    set_target_properties(zephyr_property_target PROPERTIES ameba_soc_name amebagreen2)
    set_target_properties(zephyr_property_target PROPERTIES ameba_image_name km4tz)

    # post process secondary image
    set(origin_secondary_image ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/${CONFIG_SOC_SERIES}/bin/km4ns_image2_all.bin)
    set(output_path ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project)
    set(output_prefix amebagreen2_km4ns)
    set(ameba_soc_name amebagreen2)
    set(ameba_image_name km4ns)
    set(section_name KM4NS_IMG2_XIP)
    include(${CMAKE_CURRENT_LIST_DIR}/../common/secondary_image.cmake)
  else ()
    if(CONFIG_MCUBOOT)
      # Generate container key from manifest(raw key)
      #TODO:
      #  1. support more algorithm
      #  2. move these code to common place
      if(CONFIG_AMEBA_USING_MANIFEST_KEY)
        if (CONFIG_BOOT_SIGNATURE_TYPE_ED25519)
          set(algorithm ed25519)
        elseif (CONFIG_BOOT_SIGNATURE_TYPE_ECDSA_P256)
          set(algorithm secp256r1)
        endif()
        file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project)
        file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/amebaG2/manifest.json5 DESTINATION ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project/)
        execute_process(COMMAND_ERROR_IS_FATAL ANY
          COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py encrypt topem
                  --output-file ${CONFIG_BOOT_SIGNATURE_KEY_FILE}
                  --algorithm ${algorithm}
                  --image image2
          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/amebagreen2_gcc_project
        )
      endif()
      #Invoke mcuboot bootloader generation in merge_bin.py
      list(APPEND PYTHON_SCRIPT_ARGS --mcuboot)
    endif()

    add_custom_target(raw.map ALL
      DEPENDS ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
    )
    add_custom_command(
      OUTPUT ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_NM} ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} | sort > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_OBJDUMP} -d ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.asm
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/merge_bin.py ${PYTHON_SCRIPT_ARGS}
      DEPENDS ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${CONFIG_SOC_SERIES}/manifest.json5
    )
  endif()
else()
  message(WARNING "Missing Realtek blobs. Run: `west blobs fetch hal_realtek`")
endif()