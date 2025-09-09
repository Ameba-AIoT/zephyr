/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

	SECTION_PROLOGUE(.ram.start,,SUBALIGN(32))
	{
		KEEP(*(SORT(.ram.start.text)))

		. = ALIGN(32);
	} GROUP_LINK_IN(RAMABLE_REGION)
