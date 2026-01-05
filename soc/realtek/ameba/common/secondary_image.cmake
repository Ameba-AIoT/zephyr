# Copyright (c) 2020-2025 Nordic Semiconductor ASA
# Copyright (c) 2025 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# This file includes extra build system logic for secondary image that is enabled when
# CONFIG_BOOTLOADER_MCUBOOT=y.
# This file should be invoked by including directly
# You should set these variables before include it:
#   - origin_secondary_image: origin image file path
#   - output_path: for temperary file location
#   - output_prefix: custom output prefix for binary file

#NOTE: zephyr_secondary_image_tasks refer to main image in zephyr/cmake/mcuboot.cmake
#      some mode like CONFIG_MCUBOOT_BOOTLOADER_MODE_RAM_LOAD is not support and test yet

function(zephyr_runner_file type path)
  # Property magic which makes west flash choose the signed build
  # output of a given type.
  set_target_properties(runners_yaml_props_target PROPERTIES "${type}_file" "${path}")
endfunction()

function(zephyr_secondary_image_tasks)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py pad
            --input-file ${origin_secondary_image}
            --value 0x0
            --length ${CONFIG_ROM_START_OFFSET} # 0x200 when enable mcuboot
            --from-head
            --no-align
            --output-file ${output_path}/${output_prefix}.bin
  )

  set(keyfile "${CONFIG_MCUBOOT_SIGNATURE_KEY_FILE}")
  set(keyfile_enc "${CONFIG_MCUBOOT_ENCRYPTION_KEY_FILE}")
  string(CONFIGURE "${keyfile}" keyfile)
  string(CONFIGURE "${keyfile_enc}" keyfile_enc)

  if(NOT "${CONFIG_MCUBOOT_GENERATE_UNSIGNED_IMAGE}")
    # Check for misconfiguration.
    if("${keyfile}" STREQUAL "")
      # No signature key file, no signed binaries. No error, though:
      # this is the documented behavior.
      message(WARNING "Neither CONFIG_MCUBOOT_GENERATE_UNSIGNED_IMAGE or "
                      "CONFIG_MCUBOOT_SIGNATURE_KEY_FILE are set, the generated build will not be "
                      "bootable by MCUboot unless it is signed manually/externally.")
      return()
    elseif(NOT (CONFIG_BUILD_OUTPUT_BIN OR CONFIG_BUILD_OUTPUT_HEX))
      message(FATAL_ERROR "Can't sign images for MCUboot: Neither "
                          "CONFIG_BUILD_OUTPUT_BIN nor CONFIG_BUILD_OUTPUT_HEX "
                          "is enabled, so there's nothing to sign.")
    endif()

    foreach(file keyfile keyfile_enc)
      if("${${file}}" STREQUAL "")
        continue()
      endif()

      # Find the key files in the order of preference for a simple search
      # modeled by the if checks across the various locations
      #
      #  1. absolute
      #  2. application config
      #  3. west topdir (optional when the workspace is not west managed)
      #
      if(NOT IS_ABSOLUTE "${${file}}")
        if(EXISTS "${APPLICATION_CONFIG_DIR}/${${file}}")
          set(${file} "${APPLICATION_CONFIG_DIR}/${${file}}")
        else()
          # Relative paths are relative to 'west topdir'.
          #
          # This is the only file that has a relative check to topdir likely
          # from the historical callouts to "west" itself before using
          # imgtool. So, this is maintained here for backward compatibility
          #
          if(NOT WEST OR NOT WEST_TOPDIR)
            message(FATAL_ERROR "Can't sign images for MCUboot: west workspace undefined. "
                                "To fix, ensure `west topdir` is a valid workspace directory.")
          endif()
          set(${file} "${WEST_TOPDIR}/${${file}}")
        endif()
      endif()

      if(NOT EXISTS "${${file}}")
        message(FATAL_ERROR "Can't sign images for MCUboot: can't find file ${${file}} "
                            "(Note: Relative paths are searched through "
                            "APPLICATION_CONFIG_DIR=\"${APPLICATION_CONFIG_DIR}\" "
                            "and WEST_TOPDIR=\"${WEST_TOPDIR}\")")
      endif()
      message("APPLICATION_CONFIG_DIR: ${APPLICATION_CONFIG_DIR}")
      # Add key file as CMake dependency so a file change will rerun the build
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${${file}})
    endforeach()
  endif()

  # No imgtool, no signed binaries.
  if(NOT DEFINED IMGTOOL)
    message(FATAL_ERROR "Can't sign images for MCUboot: can't find imgtool. To fix, install imgtool with pip3, or add the mcuboot repository to the west manifest and ensure it has a scripts/imgtool.py file.")
    return()
  endif()

  # Fetch devicetree details for flash and slot information
  dt_chosen(flash_node PROPERTY "zephyr,flash")
  dt_nodelabel(slot2_flash NODELABEL "slot2_partition" REQUIRED)
  dt_prop(slot_size PATH "${slot2_flash}" PROPERTY "reg" INDEX 1 REQUIRED)

  # If single slot mode, or if in firmware updater mode and this is the firmware updater image,
  # use slot 0 information
  if(NOT CONFIG_MCUBOOT_BOOTLOADER_MODE_SINGLE_APP AND (NOT CONFIG_MCUBOOT_BOOTLOADER_MODE_FIRMWARE_UPDATER OR CONFIG_MCUBOOT_APPLICATION_FIRMWARE_UPDATER)
      AND NOT CONFIG_MCUBOOT_BOOTLOADER_MODE_SINGLE_APP_RAM_LOAD)
    # Slot 1 size is used instead of slot 0 size
    set(slot_size)
    dt_nodelabel(slot3_flash NODELABEL "slot3_partition" REQUIRED)
    dt_prop(slot_size PATH "${slot3_flash}" PROPERTY "reg" INDEX 1 REQUIRED)
  endif()

  # Basic 'imgtool sign' command with known image information.
  set(imgtool_sign ${PYTHON_EXECUTABLE} ${IMGTOOL} sign
      --version ${CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION} --header-size ${CONFIG_ROM_START_OFFSET}
      --slot-size ${slot_size})

  # Arguments to imgtool.
  if(NOT CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS STREQUAL "")
    # Separate extra arguments into the proper format for adding to
    # extra_post_build_commands.
    #
    # Use UNIX_COMMAND syntax for uniform results across host
    # platforms.
    separate_arguments(imgtool_args UNIX_COMMAND ${CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS})
  else()
    set(imgtool_args)
  endif()

  if(NOT "${keyfile}" STREQUAL "")
    set(imgtool_args --key "${keyfile}" ${imgtool_args})
  endif()

  if(CONFIG_MCUBOOT_IMGTOOL_OVERWRITE_ONLY)
    # Use overwrite-only instead of swap upgrades.
    set(imgtool_args --overwrite-only --align 1 ${imgtool_args})
  else()
    dt_prop(write_block_size PATH "${flash_node}" PROPERTY "write-block-size")

    if(NOT write_block_size)
      set(write_block_size 4)
      message(WARNING "slot2_partition write block size devicetree parameter is missing, assuming write block size is 4")
    endif()

    set(imgtool_args --align ${write_block_size} ${imgtool_args})
  endif()

  # Extensionless prefix of any output file.
  set(output ${output_path}/${output_prefix})

  # List of additional build byproducts.
  set(byproducts)

  # Set up .bin outputs.
  if(CONFIG_BUILD_OUTPUT_BIN)
    list(APPEND byproducts ${output}.signed.bin)
    zephyr_runner_file(bin ${output}.signed.bin)
    set(BYPRODUCT_KERNEL_SIGNED_BIN_NAME "${output}.signed.bin"
        CACHE FILEPATH "Signed kernel bin file" FORCE
    )
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
                 ${imgtool_sign} ${imgtool_args} ${output}.bin ${output}.signed.bin)

    if(CONFIG_MCUBOOT_GENERATE_CONFIRMED_IMAGE)
      list(APPEND byproducts ${output}.signed.confirmed.bin)
      zephyr_runner_file(bin ${output}.signed.confirmed.bin)
      set(BYPRODUCT_KERNEL_SIGNED_CONFIRMED_BIN_NAME "${output}.signed.confirmed.bin"
          CACHE FILEPATH "Signed and confirmed kernel bin file" FORCE
      )
      set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
                   ${imgtool_sign} ${imgtool_args} --pad --confirm ${output}.bin
                   ${output}.signed.confirmed.bin)
    endif()

    if(NOT "${keyfile_enc}" STREQUAL "")
      list(APPEND byproducts ${output}.signed.encrypted.bin)
      set(BYPRODUCT_KERNEL_SIGNED_ENCRYPTED_BIN_NAME "${output}.signed.encrypted.bin"
          CACHE FILEPATH "Signed and encrypted kernel bin file" FORCE
      )
      set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
                   ${imgtool_sign} ${imgtool_args} --encrypt "${keyfile_enc}" ${output}.bin
                   ${output}.signed.encrypted.bin)
      set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
        COMMAND ${CMAKE_COMMAND} -E copy
                ${output}.signed.encrypted.bin
                ${CMAKE_BINARY_DIR}/images)
    endif()
  endif()

  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${CMAKE_COMMAND} -E copy
            ${output_path}/${output_prefix}.signed.bin
            ${CMAKE_BINARY_DIR}/images)

  set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts ${byproducts})
endfunction()

zephyr_secondary_image_tasks()