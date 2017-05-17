MEMORY
{
  OCRAM_L (rwx) : ORIGIN = 0x00900000, LENGTH = 128K
  OCRAM_U (rwx) : ORIGIN = 0x20910000, LENGTH = 128K
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
	} > OCRAM_L
	_etext = .;
	PROVIDE(etext = .);

	.rodata :
	{
		*(.rodata)
		*(.rodata.*)
	} > OCRAM_L

	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) }
	__exidx_start = .;
	.ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) }
	__exidx_end = .;

	.data :
	{
		__data_start = .;
		*(.data)
		*(.data.*)
	} > OCRAM_U
	_edata = .;
	PROVIDE(edata = .);

	. = ALIGN(4);
	.bss :
	{
		__bss_start  = .;
		*(.bss)
		*(COMMON)
		. = ALIGN(4);
	} > OCRAM_U
	. = ALIGN(4);
	_end = .;
	__bss_end = .;
	PROVIDE(end = .);
}
