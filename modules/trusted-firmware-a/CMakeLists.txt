# Copyright (c) 2022 Arm Limited (or its affiliates). All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_BUILD_WITH_TFA)
  set(TFA_BINARY_DIR ${CMAKE_BINARY_DIR}/tfa)

  file(MAKE_DIRECTORY ${TFA_BINARY_DIR})

  # --- AmebaSmart ATF integration ---
  # ATF source is discovered via west manifest (modules/tee/tf-a/trusted-firmware-a).
  # ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR is set automatically by Zephyr module discovery.
  if(CONFIG_SOC_SERIES_AMEBASMART)
    if(NOT DEFINED ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR)
      message(FATAL_ERROR
        "ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR not set. "
        "Ensure tf-a is listed in west manifest and run 'west update'."
      )
    endif()

    # Create external dependencies that ATF Makefile expects
    set(TFA_PROJECT_DIR ${TFA_BINARY_DIR}/project_ap)
    file(MAKE_DIRECTORY ${TFA_PROJECT_DIR}/lib/soc)
    file(MAKE_DIRECTORY ${TFA_PROJECT_DIR}/image)

    # Stub .config_ca32 (ATF includes this but doesn't need FreeRTOS vars)
    file(WRITE ${TFA_PROJECT_DIR}/.config_ca32
      "# Stub .config_ca32 for Zephyr ATF build\n"
      "# No CONFIG_CP_TEST_CA32, CONFIG_MATTER_SECURE_EN needed\n"
    )

    # AUTOCONFIG_H_DIR = ${BUILD_BASE}/../menuconfig/project_ap
    # ATF's platform_def.h includes platform_autoconf.h from this directory
    set(TFA_AUTOCONFIG_DIR ${CMAKE_BINARY_DIR}/menuconfig/project_ap)
    file(MAKE_DIRECTORY ${TFA_AUTOCONFIG_DIR})
    file(WRITE ${TFA_AUTOCONFIG_DIR}/platform_autoconf.h
      "/* Stub platform_autoconf.h for Zephyr ATF build */\n"
      "#define CONFIG_IMG2_PSRAM 1\n"
    )

    # Create symlinks that TF-A platform.mk expects:
    #   $(CURRENT_DIR)/../../common/include  → modules/tee/common/include
    #   ../../../os/os_wrapper/include       → modules/os/os_wrapper/include
    set(RTK_COMMON_INCLUDE ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/common/include)
    set(RTK_OS_WRAPPER_INCLUDE ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/common/os_wrapper/include)
    set(TFA_COMMON_LINK ${ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR}/../../common/include)
    set(TFA_OS_WRAPPER_LINK ${ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR}/../../../os/os_wrapper/include)
    file(MAKE_DIRECTORY ${ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR}/../../common)
    file(MAKE_DIRECTORY ${ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR}/../../../os/os_wrapper)
    if(NOT EXISTS ${TFA_COMMON_LINK})
      execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${RTK_COMMON_INCLUDE} ${TFA_COMMON_LINK})
    endif()
    if(NOT EXISTS ${TFA_OS_WRAPPER_LINK})
      execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${RTK_OS_WRAPPER_INCLUDE} ${TFA_OS_WRAPPER_LINK})
    endif()

    # Copy libpm.a blob (needed by sp_min-sheipa.mk)
    set(LIBPM_BLOB ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/ameba/amebasmart/lib/libpm.a)
    if(EXISTS ${LIBPM_BLOB})
      file(COPY ${LIBPM_BLOB} DESTINATION ${TFA_PROJECT_DIR}/lib/soc/)
    else()
      message(WARNING "libpm.a not found at ${LIBPM_BLOB}, ATF build may fail")
    endif()
  endif()

  include(ExternalProject)

  set(tfa_image_info MAP "name: tfa, source-dir: ${ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR}")
  build_info(images VALUE ${tfa_image_info})

  ExternalProject_Add(
    tfa
    SOURCE_DIR ${ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR}
    BINARY_DIR ${TFA_BINARY_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    BUILD_ALWAYS True
    USES_TERMINAL_BUILD True
  )

  if (CONFIG_TFA_MAKE_BUILD_TYPE_DEBUG)
    set(TFA_BUILD_DEBUG "1")
  else()
    set(TFA_BUILD_DEBUG "0")
  endif()

  if(CONFIG_SOC_SERIES_AMEBASMART)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
        COMMAND ${CMAKE_COMMAND} -E env
            make -C ${ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR}
            DEBUG=${TFA_BUILD_DEBUG}
            CROSS_COMPILE=${CROSS_COMPILE}
            PROJECT_DIR=${TFA_PROJECT_DIR}
            BUILD_BASE=${TFA_BINARY_DIR}
            BL33=${CMAKE_BINARY_DIR}/zephyr/${KERNEL_BIN_NAME}
            SOC_DIR=${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/amebasmart/source/fwlib/include
            bl1 image
    )
  else()
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
        COMMAND make -C ${ZEPHYR_TRUSTED_FIRMWARE_A_MODULE_DIR}
              DEBUG=${TFA_BUILD_DEBUG}
              CROSS_COMPILE=${CROSS_COMPILE}
              BUILD_BASE=${TFA_BINARY_DIR} PLAT=${TFA_PLAT}
              BL33=${CMAKE_BINARY_DIR}/zephyr/${KERNEL_BIN_NAME}
              all fip
    )
  endif()
endif()
