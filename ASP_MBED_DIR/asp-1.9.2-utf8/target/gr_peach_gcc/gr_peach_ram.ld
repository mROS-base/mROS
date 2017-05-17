
OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm","elf32-littlearm") 
OUTPUT_ARCH(arm)
STARTUP(start.o)

PROVIDE(hardware_init_hook = 0);
PROVIDE(software_init_hook = 0);
PROVIDE(software_term_hook = 0);

ENTRY(start);

MEMORY
{
  FLASH (rx)    : ORIGIN = 0x20000000,  LENGTH =   1M
  RRAM (xrw)     : ORIGIN = 0x20100000, LENGTH =  9M
  SDRAM (xrw)   : ORIGIN = 0x08000000,  LENGTH = 128M
}


SECTIONS
{
    .text :
    {
        __text = . ;
      	__vector_start = .	;
    	PROVIDE (_vector_start = .);
        *(vector)
        start.o(.text*)
        . = 0x400;			/* IVT table offset */
        *(.ivt)
        *(.boot_data)
        *(.dcd)
	    . = ALIGN(32);
        *(.text*)
        *(.glue_7t)
        *(.glue_7)
    } >FLASH

	.ARM.extab : 
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} >FLASH

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} >FLASH
	__exidx_end = .;

    _etext = .	;
    PROVIDE (etext = .)	;
    .rodata : 
    { 
       *(.rodata*) 
       . = ALIGN(4);
       __rodata_end = . ;
    } >FLASH
    __rom_size = . + SIZEOF(.data) - __text;
    PROVIDE (_rom_size = . + SIZEOF(.data) - __text);

    . = ALIGN(4);
    .data   :  AT(__rodata_end) 
    {
        __data_start = . ;
        *(.data*)
    }> RRAM
    __idata_start = LOADADDR(.data);
    __idata_end = __idata_start + SIZEOF(.data);

    _edata  =  . ;
    PROVIDE (edata = .);

    .bss       :
    {
      __bss_start = .	;
      *(.bss*)
      *(COMMON)
   . = ALIGN(32 / 8);
    }> RRAM

    .heap :
    {
        __end__ = .;
        end = __end__;
        *(.heap*)
        __HeapLimit = .;
    } > RRAM

   . = ALIGN(32 / 8);
     _end = .		;
    __bss_end = .;
    PROVIDE (end = .)	;
    .comment 0 : { *(.comment) }

  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to 
     the beginning of the section so we begin them at 0.  */

  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }

  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }

  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }

  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }

  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
}
