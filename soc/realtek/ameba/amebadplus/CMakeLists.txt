# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_include_directories(.)

if(CONFIG_MCUBOOT)
  zephyr_linker_sources(ROM_START SORT_KEY 0x0 loader_section.ld)
  zephyr_linker_sources(RAM_SECTIONS SORT_KEY 0x0 loader_ram_section.ld)
  zephyr_sources(mcuboot_action_hook.c)
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

file(GLOB BIN_FILES "${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/ameba/${CONFIG_SOC_SERIES}/bin/*.bin")

if(BIN_FILES)
  if (CONFIG_BOOTLOADER_MCUBOOT)
  # 1. Create directories
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/amebadplus_gcc_project)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/amebadplus_gcc_project/km4)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/amebadplus_gcc_project/km0)

     # 2. Copy configuration files
    file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/amebadplus/manifest.json5 DESTINATION ${CMAKE_BINARY_DIR}/amebadplus_gcc_project/)
    file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/amebadplus/ameba_layout.ld DESTINATION ${CMAKE_BINARY_DIR}/amebadplus_gcc_project/)

    # 3. Format manifest
    execute_process(COMMAND_ERROR_IS_FATAL ANY
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py helper manifest-fmt
              --output-file ${CMAKE_BINARY_DIR}/amebadplus_gcc_project/manifest_formatted.json
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/amebadplus_gcc_project
    )

     # 4. Set debug file generation
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${CMAKE_NM} ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} | sort > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_OBJDUMP} -d ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.asm
      COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/images"
    )

    # 5. Configure PRIMARY image (KM4)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts ${KERNEL_NAME}.raw.map ${KERNEL_NAME}.asm)

    # post process primary image
    #NOTE: Because extra_post_build_commands added in soc is executed before that in ${ZEPHYR_BASE}/cmake/mcuboot.cmake,
    #      so mcuboot.cmake is used to wrapper ${ZEPHYR_BASE}/cmake/mcuboot.cmake with some extra extra_post_build_commands
    #      that needed to be executed exactly after ${ZEPHYR_BASE}/cmake/mcuboot.cmake
    set_target_properties(zephyr_property_target PROPERTIES SIGNING_SCRIPT
      ${CMAKE_CURRENT_LIST_DIR}/../common/primary_image.cmake)
    set_target_properties(zephyr_property_target PROPERTIES ameba_output_prefix amebadplus_app)
    set_target_properties(zephyr_property_target PROPERTIES ameba_soc_name amebadplus)
    set_target_properties(zephyr_property_target PROPERTIES origin_secondary_image
      ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/ameba/${CONFIG_SOC_SERIES}/bin/km0_image2_all.bin)
  else ()
    if(CONFIG_MCUBOOT)
      # Generate container key from manifest(raw key)
      if(CONFIG_AMEBA_USING_MANIFEST_KEY)
        if (CONFIG_BOOT_SIGNATURE_TYPE_ED25519)
          set(algorithm ed25519)
        elseif (CONFIG_BOOT_SIGNATURE_TYPE_ECDSA_P256)
          set(algorithm secp256r1)
        endif()
        file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/amebadplus_gcc_project)
        file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/amebadplus/manifest.json5 DESTINATION ${CMAKE_BINARY_DIR}/amebadplus_gcc_project/)
        execute_process(COMMAND_ERROR_IS_FATAL ANY
          COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py encrypt topem
                  --output-file ${CONFIG_BOOT_SIGNATURE_KEY_FILE}
                  --algorithm ${algorithm}
                  --image image2
          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/amebadplus_gcc_project
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
