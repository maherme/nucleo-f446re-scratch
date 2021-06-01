/* Entry Point */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH(RX) : ORIGIN = 0x08000000, LENGTH = 512K
    SRAM(RWX) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
    .text :
    {
        KEEP (*(.isr_vector))
        *(.text)
        *(.text.*)
        KEEP (*(.init))
    	KEEP (*(.fini))
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
        _etext = .;
    }> FLASH

    _la_data = LOADADDR(.data);

    .data :
    {
        _sdata = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    }> SRAM AT> FLASH

    .bss :
    {
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
        . = ALIGN(4);
        end = .;
        __end__ = .;
    }> SRAM
}
