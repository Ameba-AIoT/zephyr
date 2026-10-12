# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# Sign the Flash Loader when manifest.json5 enables secure boot, same as the SDK.

set(axf2bin "${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py")
set(soc_data_dir "${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${SOC_SERIES}")
set(floader_work_dir "${CMAKE_BINARY_DIR}/${SOC_SERIES}_gcc_project")

file(GLOB floader_blob
     "${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/ameba/${SOC_SERIES}/bin/floader_*.bin")
list(LENGTH floader_blob floader_blob_count)

if(NOT floader_blob_count EQUAL 1)
  message(FATAL_ERROR
          "Expected one Flash Loader blob for ${SOC_SERIES}, found ${floader_blob_count}")
endif()

get_filename_component(floader_name ${floader_blob} NAME)
set(floader_signed "${CMAKE_BINARY_DIR}/images/${floader_name}")

# Re-run CMake when sboot_enable may have changed
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${soc_data_dir}/manifest.json5")

file(MAKE_DIRECTORY "${floader_work_dir}")
file(COPY "${soc_data_dir}/manifest.json5" DESTINATION "${floader_work_dir}")
file(COPY "${soc_data_dir}/ameba_layout.ld" DESTINATION "${floader_work_dir}")

execute_process(
  COMMAND ${PYTHON_EXECUTABLE} ${axf2bin}
          --post-build-dir ${floader_work_dir}
          helper
          query-manifest
          --key sboot_enable
  OUTPUT_VARIABLE sboot_enable
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE sboot_query_result
)

if(NOT sboot_query_result EQUAL 0)
  message(FATAL_ERROR "Failed to query sboot_enable from ${soc_data_dir}/manifest.json5")
endif()

if(NOT sboot_enable STREQUAL "ON")
  # The MCUboot flows never wipe images/, so drop one left by an earlier build
  file(REMOVE "${floader_signed}")
  return()
endif()

# raw.map is written after images/, which merge_bin.py wipes and refills
add_custom_command(
  OUTPUT "${floader_signed}"
  # Strip the trailing 4 KB manifest
  COMMAND ${PYTHON_EXECUTABLE} ${axf2bin}
          --post-build-dir ${floader_work_dir}
          cut
          -i ${floader_blob}
          -o ${floader_work_dir}/floader_body.bin
          -l 4096
          -B
  # Append a newly signed manifest
  COMMAND ${PYTHON_EXECUTABLE} ${axf2bin}
          --post-build-dir ${floader_work_dir}
          fw_pack
          --imgtool-floader ${floader_work_dir}/floader_body.bin
          -o ${floader_work_dir}/${floader_name}
  COMMAND ${CMAKE_COMMAND} -E copy
          ${floader_work_dir}/${floader_name}
          ${floader_signed}
  DEPENDS "${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map"
          "${floader_blob}"
          "${soc_data_dir}/manifest.json5"
  COMMENT "Signing the Flash Loader for secure boot"
)
add_custom_target(floader_sboot ALL DEPENDS "${floader_signed}")
