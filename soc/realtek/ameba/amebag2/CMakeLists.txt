# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_include_directories(.)

# ld.bfd 2.38 (shipped with asdk-12.3.1) incorrectly accounts for
# DWARF debug sections (VMA=0) when computing FLASH region overflow,
# causing false "section will not fit in region FLASH" errors.
# Discard debug sections explicitly in the linker script instead of
# using --strip-debug (which also removes .intList needed by gen_isr_tables).
# The DISCARD entries are injected via zephyr_linker_sources().
# Note: CONFIG_FLASH_SIZE is overridden to 32768 in Kconfig.defconfig to
# work around ld.bfd 2.38 bug where debug sections (VMA=0) are incorrectly
# counted toward FLASH region overflow. See Kconfig.defconfig for details.

if(CONFIG_MCUBOOT)
  zephyr_linker_sources(ROM_START SORT_KEY 0x0 loader_section.ld)
  zephyr_sources(${CMAKE_CURRENT_LIST_DIR}/../common/mcuboot_action_hook.c)
else()
  zephyr_sources(soc.c)
  zephyr_library_sources_ifdef(CONFIG_PM ${CMAKE_CURRENT_LIST_DIR}/../common/pm.c)
endif()

zephyr_linker_sources(RAMFUNC_SECTION ${CMAKE_CURRENT_LIST_DIR}/../common/ram_function_section.ld)
zephyr_linker_sources(RAM_SECTIONS SORT_KEY 0x0 boot_section.ld)
zephyr_linker_sources(SECTIONS psram_section.ld)
# ld.bfd 2.38 (asdk-12.3.1) places .intList* as orphan sections before the
# SORT_KEY 0x0 so it is processed first, before rom_start, preventing orphan
# placement in FLASH. This is only needed for ld.bfd >= 2.38.
# Activate the pre-existing discard_debug.ld: discard DWARF debug sections so
# ld.bfd 2.38 does not count them toward FLASH region overflow.
zephyr_linker_sources(SECTIONS ${CMAKE_CURRENT_LIST_DIR}/discard_debug.ld)
set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/../common/linker_cortex_m.ld CACHE INTERNAL "")

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
  if (CONFIG_BOOTLOADER_MCUBOOT OR CONFIG_TFM_BL2)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project/km4tz)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project/km4ns)
    file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${SOC_SERIES}/manifest.json5 DESTINATION ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project/)
    file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${SOC_SERIES}/ameba_layout.ld DESTINATION ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project/)
    execute_process(COMMAND_ERROR_IS_FATAL ANY
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py helper manifest-fmt
              --output-file ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project/manifest_formatted.json
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project
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
      ${CMAKE_CURRENT_LIST_DIR}/../common/mcuboot_app_image.cmake)
    set_target_properties(zephyr_property_target PROPERTIES ameba_output_prefix app)
    set_target_properties(zephyr_property_target PROPERTIES ameba_soc_name ${SOC_SERIES})
    set_target_properties(zephyr_property_target PROPERTIES origin_secondary_image
      ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/ameba/${CONFIG_SOC_SERIES}/bin/km4ns_image2_all.bin)
  elseif (CONFIG_MCUBOOT)
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
      file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project)
      file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${SOC_SERIES}/manifest.json5 DESTINATION ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project/)
      execute_process(COMMAND_ERROR_IS_FATAL ANY
        COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py encrypt topem
                --output-file ${CONFIG_BOOT_SIGNATURE_KEY_FILE}
                --algorithm ${algorithm}
                --image image2
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project
      )
    endif()
    #Invoke mcuboot bootloader generation using cmake
    include(${CMAKE_CURRENT_LIST_DIR}/../common/mcuboot_image.cmake)
  else()
    add_custom_target(raw.map ALL
      DEPENDS ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
    )
    add_custom_command(
      OUTPUT ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_NM} ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} | sort > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_OBJDUMP} -d ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.asm
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/merge_bin.py ${PYTHON_SCRIPT_ARGS}
      DEPENDS
        ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME}
        ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${CONFIG_SOC_SERIES}/manifest.json5
        ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/merge_bin.py
    )
  endif()
else()
  message(WARNING "Missing Realtek blobs. Run: `west blobs fetch hal_realtek`")
endif()
