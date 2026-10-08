# Copyright (c) 2025 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# This file includes extra build system logic for mcuboot image that is enabled when
# CONFIG_MCUBOOT=y.

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/${CONFIG_SOC_SERIES}_gcc_project)
set(td ${CMAKE_BINARY_DIR}/${CONFIG_SOC_SERIES}_gcc_project)

# Copy layout and manifest as merge_bin.py did
file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${CONFIG_SOC_SERIES}/manifest.json5 DESTINATION ${td}/)
file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${CONFIG_SOC_SERIES}/ameba_layout.ld DESTINATION ${td}/)

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${CMAKE_NM} ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} | sort > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
    COMMAND ${CMAKE_OBJDUMP} -d ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.asm
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/images"
)

if(CONFIG_SOC_SERIES_AMEBAG2)
    set(boot_text_start "__km4tz_boot_text_start__")
    # AP_BOOT_INDEX: the ROM takes the loader entry from this header
    set(header_value "0x01010101")
    set(ram_1_symbol "${boot_text_start}")
else()
    set(boot_text_start "__km4_boot_text_start__")
    set(header_value "0xFFFFFFFF")
    set(ram_1_symbol "__ram_start_table_start__")
endif()

# Emit only the sub-images the ROM walks (amebasmart 1, amebadplus/amebagreen2 2).
# Secure boot hashes the sub-images the ROM walked, so an extra one breaks it.
if(CONFIG_SOC_SERIES_AMEBASMART)
    set(ram_1_cmds "")
    set(image1_parts ${td}/xip_boot_prepend.bin)
else()
    set(ram_1_cmds
        # 5. extract ram_1
        COMMAND ${CMAKE_OBJCOPY} -O binary --only-section=.ram_image1.entry
            ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} ${td}/ram_1.bin
        # 6. prepend header to ram_1
        COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py --post-build-dir ${td} prepend_header
            --output-file ${td}/ram_1_prepend.bin
            --input-file ${td}/ram_1.bin
            --map-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
            --symbol ${ram_1_symbol}
    )
    set(image1_parts ${td}/xip_boot_prepend.bin ${td}/ram_1_prepend.bin)
endif()

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    # 1. extract xip_all
    COMMAND ${CMAKE_OBJCOPY} -O binary --remove-section=.ram_image1.entry
        ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} ${td}/xip_all.bin
    # 2. cut by map
    #NOTE: --post-build-dir must be passed to every axf2bin.py call: without it the
    #      script falls back to cwd (${ZEPHYR_BINARY_DIR}) for project detection, which
    #      misparses build directories named build_* as a standalone-SDK SoC project.
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py --post-build-dir ${td} helper cut-by-map
        --input-file ${td}/xip_all.bin
        --output-file ${td}/xip_boot.bin
        --map-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
        --start-sym "__rom_region_start"
        --end-sym ${boot_text_start}
    # 3. pad 32
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py --post-build-dir ${td} pad
        --input-file ${td}/xip_boot.bin
        --length 32
    # 4. prepend header to xip_boot
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py --post-build-dir ${td} prepend_header
        --output-file ${td}/xip_boot_prepend.bin
        --input-file ${td}/xip_boot.bin
        --map-file ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
        --symbol ${boot_text_start}
        --boot-index ${header_value}
    # 5-6. extract ram_1 and prepend its header (only where the ROM walks 2 sub-images)
    ${ram_1_cmds}
    # 7. concat
    COMMAND ${CMAKE_COMMAND} -E cat
        ${image1_parts}
        > ${td}/xip_ram_boot.bin
    # 8. fw pack
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py --post-build-dir ${td} fw_pack
        --output-file ${td}/boot.bin
        --image1 ${td}/xip_ram_boot.bin
    # 9. copy to images
    COMMAND ${CMAKE_COMMAND} -E copy
        ${td}/boot.bin
        ${CMAKE_BINARY_DIR}/images/boot.bin
)
