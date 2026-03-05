/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

. = ALIGN(32);
*(.non.dram.text*)
*(.sram.only.text*)
*(.sram.only.data*)
. = ALIGN(32);
