MEMORY
{
  TCML (rwx)  : ORIGIN = 0x00000000, LENGTH = 32K
  TCMU (rwx)  : ORIGIN = 0x20000000, LENGTH = 32K

  OCRAM_L (rwx) : ORIGIN = 0x00900000, LENGTH = 128K
  OCRAM_U (rwx) : ORIGIN = 0x20900000, LENGTH = 128K
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
	} > TCML
	_etext = .;
	PROVIDE(etext = .);

	.rodata :
	{
		*(.rodata)
		*(.rodata.*)
	} >  TCML

	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) }
	__exidx_start = .;
	.ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) }
	__exidx_end = .;

	.data :
	{
		__data_start = .;
		*(.data)
		*(.data.*)
	} > TCMU
	_edata = .;
	PROVIDE(edata = .);

	. = ALIGN(4);

	.bss :
	{
		__bss_start  = .;
		*(.bss)
		*(COMMON)
		. = ALIGN(4);
	} > TCMU
	. = ALIGN(4);
	_end = .;
	__bss_end = .;
	PROVIDE(end = .);
}
