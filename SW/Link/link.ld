/* SW/Link/link.ld */
INCLUDE memmap.ldh

ENTRY(_start)

MEMORY
{
  RAM (rwx) : ORIGIN = MRV_RAM_BASE, LENGTH = MRV_RAM_SIZE
}

SECTIONS
{
  . = ORIGIN(RAM);

  .text : ALIGN(4) {
    *(.text .text.*)
    *(.init)
    *(.fini)
  } > RAM

  .rodata : ALIGN(4) {
    *(.rodata .rodata.*)
  } > RAM

  .data : ALIGN(4) {
    *(.data .data.*)
  } > RAM

  .bss : ALIGN(4) {
    __bss_start = .;
    *(.bss .bss.*)
    *(COMMON)
    __bss_end = .;
  } > RAM

  . = ALIGN(4);
  _end = .;

  /* Stack grows down, stack top at end of RAM */
  _stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
