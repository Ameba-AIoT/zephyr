# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_include_directories(.)

if(CONFIG_MCUBOOT OR CONFIG_BOOTLOADER_MCUBOOT)
  zephyr_linker_sources_ifdef(CONFIG_MCUBOOT ROM_START SORT_KEY 0x0 loader_section.ld)
  zephyr_sources(
    soc_loader.c
    ../common/loader.c
  )
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

if(CONFIG_MCUBOOT OR CONFIG_BOOTLOADER_MCUBOOT)
    list(APPEND PYTHON_SCRIPT_ARGS --mcuboot)
endif()

if(CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION)
    list(APPEND PYTHON_SCRIPT_ARGS --sign-version "${CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION}")
endif()

#  west sign

file(GLOB BIN_FILES "${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/${CONFIG_SOC_SERIES}/bin/*.bin")

if(BIN_FILES)
  add_custom_target(zephyr.raw.map ALL
    DEPENDS ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
  )

  add_custom_command(
    OUTPUT ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
    COMMAND ${CMAKE_NM} ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} | sort > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
    COMMAND ${CMAKE_OBJDUMP} -d ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.asm
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/merge_bin.py ${PYTHON_SCRIPT_ARGS}
    DEPENDS ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${CONFIG_SOC_SERIES}/manifest.json5
  )
else()
  message(WARNING "Missing Realtek blobs. Run: `west blobs fetch hal_realtek`")
endif()
