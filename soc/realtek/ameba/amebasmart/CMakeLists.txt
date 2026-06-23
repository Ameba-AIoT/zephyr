# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(soc.c)
zephyr_library_sources_ifdef(CONFIG_ARM_AARCH32_MMU mmu_regions.c)

zephyr_include_directories(.)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_a_r/scripts/linker.ld CACHE INTERNAL "")

# amebasmart HAL blobs require the v7-a+neon-fp16/fpu hard-float multilib.
# TOOLCHAIN_C_FLAGS uses -mfpu=neon-fp-armv8 which has no matching multilib in
# asdk-10.3.1, so gcc/target.cmake and lib/libc/newlib set LIBC_LIBRARY_DIR to
# the soft-float base dir.  Fix: resolve both libgcc and libc/libm paths using
# the correct -march=armv7-a+neon-fp16 -mfloat-abi=hard flags, then replace the
# soft-float -L that newlib already added to zephyr_interface.
execute_process(
  COMMAND ${CMAKE_C_COMPILER} -march=armv7-a+neon-fp16 -mfloat-abi=hard -print-libgcc-file-name
  OUTPUT_VARIABLE _libgcc_file
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
get_filename_component(_libgcc_dir ${_libgcc_file} DIRECTORY)
zephyr_link_libraries(-L${_libgcc_dir})

# Replace the soft-float libc/libm -L with the v7-a+neon-fp16/fpu hard-float one.
# LIBC_LIBRARY_DIR is set by gcc/target.cmake; SYSROOT_DIR by gnuarmemb/generic.cmake.
if(LIBC_LIBRARY_DIR AND SYSROOT_DIR)
  execute_process(
    COMMAND ${CMAKE_C_COMPILER} -march=armv7-a+neon-fp16 -mfloat-abi=hard -print-multi-directory
    OUTPUT_VARIABLE _hf_multilib_subdir
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(_hf_libc_dir "${SYSROOT_DIR}/lib/${_hf_multilib_subdir}")
  if(EXISTS "${_hf_libc_dir}")
    get_property(_iface_libs TARGET zephyr_interface PROPERTY INTERFACE_LINK_LIBRARIES)
    list(FIND _iface_libs "-L${LIBC_LIBRARY_DIR}" _sf_idx)
    if(_sf_idx GREATER_EQUAL 0)
      list(REMOVE_AT _iface_libs ${_sf_idx})
      list(INSERT _iface_libs ${_sf_idx} "-L${_hf_libc_dir}")
      set_property(TARGET zephyr_interface PROPERTY INTERFACE_LINK_LIBRARIES "${_iface_libs}")
    endif()
  endif()
endif()

zephyr_linker_sources(RAM_SECTIONS SORT_KEY 0x0 boot_section.ld)

set(PYTHON_SCRIPT_ARGS
    --soc ${CONFIG_SOC_SERIES}
    --bin-file ${ZEPHYR_BINARY_DIR}/${KERNEL_BIN_NAME}
    --out-dir ${CMAKE_BINARY_DIR}
    --module-dir ${ZEPHYR_HAL_REALTEK_MODULE_DIR}
)

if(CONFIG_BT_COEXIST)
    list(APPEND PYTHON_SCRIPT_ARGS --bt-coexist)
endif()

file(GLOB BIN_FILES "${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/ameba/${CONFIG_SOC_SERIES}/bin/*.bin")

if(BIN_FILES)
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
else()
  message(WARNING "Missing Realtek blobs. Run: `west blobs fetch hal_realtek`")
endif()
