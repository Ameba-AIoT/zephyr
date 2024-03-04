# SPDX-License-Identifier: Apache-2.0

set(EMU_PLATFORM qemu)

set(QEMU_CPU_TYPE_${ARCH} real-m300v)
set(QEMU_FLAGS_${ARCH}
  -cpu ${QEMU_CPU_TYPE_${ARCH}}
  -machine sheipa
  -nographic
  )

if (CONFIG_BUILD_WITH_TFM)
  # Override the binary used by qemu, to use the combined
  # TF-M (Secure) & Zephyr (Non Secure) image (when running
  # in-tree tests).
  set(QEMU_KERNEL_OPTION "-device;loader,file=${CMAKE_BINARY_DIR}/tfm_merged.hex")
endif()
