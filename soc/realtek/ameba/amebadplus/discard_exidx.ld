/* ld.bfd 2.35 workaround: discard .ARM.exidx from libgcc before it reaches
 * the ROMABLE_REGION GROUP. This prevents the "rom_start has both ordered
 * and unordered sections" error caused by libgcc _udivmoddi4.o having
 * .ARM.exidx (sorted) while other objects have unsorted .text sections.
 * C++ exception unwinding is not used in this firmware. */
/DISCARD/ : { *(.ARM.exidx*) *(gnu.linkonce.armexidx.*) }
