MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K
  SRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 96K
}

PROVIDE(hardware_init_hook = 0);
PROVIDE(software_init_hook = 0);
PROVIDE(software_term_hook = 0);

ENTRY(_kernel__start)

SECTIONS
{
	.text :
	{
		__text = .;
		*(.vector)
		*(.text)
		*(.text.*)
	} > FLASH
	_etext = .;
	PROVIDE(etext = .);

	.rodata :
	{
		*(.rodata)
		*(.rodata.*)
	} > FLASH

	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) }
	__exidx_start = .;
	.ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) }
	__exidx_end = .;

	__idata_start = .;
	.data : AT (__idata_start)
	{
		__data_start = .;
		*(.data)
		*(.data.*)
	} > SRAM
	__idata_end = __idata_start + SIZEOF(.data);
	_edata = .;
	PROVIDE(edata = .);

	. = ALIGN(4);

	.bss :
	{
		__bss_start  = .;
		*(.bss)
		*(COMMON)
		. = ALIGN(4);
	} > SRAM
	. = ALIGN(4);
	_end = .;
	__bss_end = .;
	PROVIDE(end = .);
}
