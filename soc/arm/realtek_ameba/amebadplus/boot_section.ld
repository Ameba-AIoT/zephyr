/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_REALTEK_IMAGE_HEADER_NEED_TODO)
GROUP_START(KM4_IMG2_ENTRY)
	SECTION_PROLOGUE(.ram_image2.entry, (NOLOAD), SUBALIGN(32))
	{
		__image2_entry_func__ = .;
		KEEP(*(SORT(.image2.entry.data*)))
		. = ALIGN(32);
	} GROUP_LINK_IN(KM4_IMG2_ENTRY)

GROUP_END(KM4_IMG2_ENTRY)
#endif


	SECTION_PROLOGUE(.ram_image2.entry,,SUBALIGN(32))
	{
		__image2_entry_func__ = .;
		KEEP(*(SORT(.image2.entry.data*)))
		. = ALIGN(32);
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

