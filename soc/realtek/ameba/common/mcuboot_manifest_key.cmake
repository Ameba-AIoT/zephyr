# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# Export manifest.json5's image2 (application) sboot_private_key to the PEM at
# CONFIG_BOOT_SIGNATURE_KEY_FILE. Enabled by CONFIG_AMEBA_USING_MANIFEST_KEY.

if(CONFIG_BOOT_SIGNATURE_TYPE_ED25519)
  set(algorithm ed25519)
elseif(CONFIG_BOOT_SIGNATURE_TYPE_ECDSA_P256)
  set(algorithm secp256r1)
else()
  # axf2bin.py "encrypt topem" exports no other algorithm.
  message(FATAL_ERROR "CONFIG_AMEBA_USING_MANIFEST_KEY needs "
                      "SB_CONFIG_BOOT_SIGNATURE_TYPE_ED25519 or _ECDSA_P256")
endif()

# The key file is overwritten, and it defaults into the MCUboot repository.
cmake_path(SET key_file NORMALIZE "${CONFIG_BOOT_SIGNATURE_KEY_FILE}")
cmake_path(SET mcuboot_dir NORMALIZE "${ZEPHYR_MCUBOOT_MODULE_DIR}")
cmake_path(IS_PREFIX mcuboot_dir "${key_file}" NORMALIZE key_in_mcuboot_tree)

if(key_in_mcuboot_tree)
  message(FATAL_ERROR "SB_CONFIG_BOOT_SIGNATURE_KEY_FILE must point outside "
                      "${mcuboot_dir}, it is overwritten: ${key_file}")
endif()

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/${CONFIG_SOC_SERIES}_gcc_project)
file(COPY ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/${CONFIG_SOC_SERIES}/manifest.json5
     DESTINATION ${CMAKE_BINARY_DIR}/${CONFIG_SOC_SERIES}_gcc_project/)
execute_process(COMMAND_ERROR_IS_FATAL ANY
  COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/axf2bin.py encrypt topem
          --output-file ${key_file}
          --algorithm ${algorithm}
          --image image2
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${CONFIG_SOC_SERIES}_gcc_project
)
