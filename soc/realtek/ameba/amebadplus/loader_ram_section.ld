/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

SECTION_PROLOGUE(.ram_image1.entry,,SUBALIGN(32))
{
    __ram_image1_text_start__ = .;
    __ram_start_table_start__ = .;
    KEEP(*(SORT_BY_NAME(.image1.entry.data*)))
    KEEP(*(.image1.validate.rodata*))
    *(.image1.export.symb)
    KEEP(*(SORT_BY_NAME(.image1.ram.text*)))

    . = ALIGN(32);
    __ram_image1_text_end__ = .;
} GROUP_LINK_IN(KM4_IMG1_ENTRY AT> FLASH)
