/*
 * Discard DWARF debug sections explicitly so that ld.bfd 2.38
 * (asdk-12.3.1-4600) does not count them when computing FLASH region size.
 * Without this, ld.bfd 2.38 incorrectly flags a "region FLASH overflowed"
 * error even though the actual code + data fits within the region.
 */
/DISCARD/ : {
    *(.debug_info)
    *(.debug_abbrev)
    *(.debug_aranges)
    *(.debug_ranges)
    *(.debug_line)
    *(.debug_line_str)
    *(.debug_str)
    *(.debug_loc)
    *(.debug_loclists)
    *(.debug_frame)
    *(.debug_macinfo)
    *(.debug_macro)
    *(.debug_pubtypes)
    *(.debug_pubnames)
    *(.debug_weaknames)
    *(.debug_funcnames)
    *(.debug_typenames)
    *(.debug_varnames)
    *(.debug_addr)
    *(.debug_rnglists)
    *(.gnu.linkonce.wi.*)
}
