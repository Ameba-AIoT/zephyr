/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

SECTION_PROLOGUE(ameba_cmd_table,,)
{
	. = ALIGN(4);
	__cmd_table_start__ = .;
	KEEP(*(.cmd.table.data*))
	__cmd_table_end__ = .;
} GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
