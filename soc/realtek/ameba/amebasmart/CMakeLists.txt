# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

# ---------------------------------------------------------------------------
# Linker script and SoC sources: branch on CA32 (app) vs KM4 (MCUboot)
# ---------------------------------------------------------------------------

if(CONFIG_MCUBOOT)
  # KM4 Cortex-M55: MCUboot bootloader domain
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/../common/linker_cortex_m.ld CACHE INTERNAL "")

  zephyr_linker_sources(ROM_START SORT_KEY 0x0 loader_section.ld)
  zephyr_linker_sources(RAM_SECTIONS SORT_KEY 0x0 boot_section_km4.ld)
  zephyr_linker_sources(RAM_SECTIONS SORT_KEY 0x1 km4_msp_stack.ld)

  zephyr_sources(${CMAKE_CURRENT_LIST_DIR}/../common/mcuboot_action_hook.c)

else()
  # CA32 Cortex-A32: Zephyr application domain
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

  # CA32 app sources
  zephyr_sources(soc.c)
  zephyr_library_sources_ifdef(CONFIG_ARM_AARCH32_MMU mmu_regions.c)

  # CA32 uses its own PM path (clock-gating via LP IPC + power-gating via PSCI),
  # not the Cortex-M s2ram flow in ../common/pm.c. The low-level PM primitives
  # (cpu_suspend/resume, GIC save/restore, PSCI PG entry, CG/PG lib, CPU1
  # backup/restore) are in lib_pmc.a (see the HAL CMakeLists).
  #   pm.c        - Zephyr PM hooks + PG/CG orchestration + SMP + wake sources
  #   ameba_pmc.c - SOCPS_SleepCG wrapper + sleep_param
  zephyr_library_sources_ifdef(CONFIG_PM pm.c ameba_pmc.c)

  zephyr_linker_sources(RAM_SECTIONS SORT_KEY 0x0 boot_section.ld)

endif()

# ---------------------------------------------------------------------------
# Post-build image packaging
# ---------------------------------------------------------------------------

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
  if(CONFIG_MCUBOOT)
    # ---- KM4 MCUboot bootloader build ----
    if(CONFIG_AMEBA_USING_MANIFEST_KEY)
      if(CONFIG_BOOT_SIGNATURE_TYPE_ED25519)
        set(algorithm ed25519)
      elseif(CONFIG_BOOT_SIGNATURE_TYPE_ECDSA_P256)
        set(algorithm secp256r1)
      endif()
      file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project)
      file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${SOC_SERIES}/manifest.json5
           DESTINATION ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project/)
      execute_process(COMMAND_ERROR_IS_FATAL ANY
        COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py encrypt topem
                --output-file ${CONFIG_BOOT_SIGNATURE_KEY_FILE}
                --algorithm ${algorithm}
                --image image2
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project
      )
    endif()
    include(${CMAKE_CURRENT_LIST_DIR}/../common/mcuboot_image.cmake)

  elseif(CONFIG_BOOTLOADER_MCUBOOT)
    # ---- CA32 app build (MCUboot-aware, to be signed by imgtool) ----
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project)
    file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${SOC_SERIES}/manifest.json5
         DESTINATION ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project/)
    file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${SOC_SERIES}/ameba_layout.ld
         DESTINATION ${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project/)
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
    set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts
      ${KERNEL_NAME}.raw.map ${KERNEL_NAME}.asm)
    set_target_properties(zephyr_property_target PROPERTIES SIGNING_SCRIPT
      ${CMAKE_CURRENT_LIST_DIR}/../common/mcuboot_app_image.cmake)
    set_target_properties(zephyr_property_target PROPERTIES ameba_output_prefix app)
    set_target_properties(zephyr_property_target PROPERTIES ameba_soc_name ${SOC_SERIES})
    set_target_properties(zephyr_property_target PROPERTIES origin_secondary_image
      ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/ameba/${SOC_SERIES}/bin/km0_image2_all.bin)

  else()
    # ---- Regular CA32 app build (no MCUboot) ----
    add_custom_target(raw.map ALL
      DEPENDS ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
    )
    add_custom_command(
      OUTPUT ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_NM} ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} | sort > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_OBJDUMP} -d ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.asm
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/merge_bin.py ${PYTHON_SCRIPT_ARGS}
      DEPENDS ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME}
              ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${CONFIG_SOC_SERIES}/manifest.json5
    )
  endif()
else()
  message(WARNING "Missing Realtek blobs. Run: `west blobs fetch hal_realtek`")
endif()
