# SPDX-License-Identifier: Apache-2.0
set(SUPPORTED_EMU_PLATFORMS qemu)

set(QEMU_binary_suffix riscv32)
set(QEMU_CPU_TYPE_${ARCH} riscv32)

set(QEMU_FLAGS_${ARCH}
  -nographic
  -machine sifive_e
)

board_set_debugger_ifnset(qemu)
