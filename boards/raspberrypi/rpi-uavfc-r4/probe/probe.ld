/*
 * RP2350 bare-metal probe linker script (phase 1).
 *
 * Preprocessed by the Makefile with:
 *   -DFLASH_ORIGIN=0x10000000 -DFLASH_LENGTH=128K   (bootloader probe)
 *   -DFLASH_ORIGIN=0x10020000 -DFLASH_LENGTH=3904K  (application probe)
 *
 * Physical SRAM is 0x20000000-0x2007ffff (512K striped, ds 2.2.3) plus two
 * 4K non-striped banks at 0x20080000 and 0x20081000.  The probe reserves the
 * first and last eight bytes of the striped region and of each 4K bank as
 * test storage for the SRAM boundary check.  These reservations are probe
 * only; the production NuttX scripts use the full regions.
 */

#ifndef FLASH_ORIGIN
#error FLASH_ORIGIN not defined
#endif
#ifndef FLASH_LENGTH
#error FLASH_LENGTH not defined
#endif
#ifndef STACK_SIZE
#define STACK_SIZE 8K
#endif

MEMORY
{
    flash (rx)           : ORIGIN = FLASH_ORIGIN, LENGTH = FLASH_LENGTH
    sram_lo_test (rw)    : ORIGIN = 0x20000000, LENGTH = 8
    sram (rwx)           : ORIGIN = 0x20000008, LENGTH = 512K - 16
    sram_hi_test (rw)    : ORIGIN = 0x2007fff8, LENGTH = 8
    scratch_x_test (rw)  : ORIGIN = 0x20080000, LENGTH = 8
    scratch_x (rwx)      : ORIGIN = 0x20080008, LENGTH = 4K - 8
    scratch_y (rwx)      : ORIGIN = 0x20081000, LENGTH = 4K - 8
    scratch_y_test (rw)  : ORIGIN = 0x20081ff8, LENGTH = 8
}

ENTRY(Reset_Handler)

/* Initial MSP: top of the usable striped region, 8-byte aligned (0x2007fff8). */
_stack_top    = ORIGIN(sram) + LENGTH(sram);
_stack_bottom = _stack_top - STACK_SIZE;

SECTIONS
{
    .text :
    {
        _stext = .;
        KEEP(*(.vectors))
        _image_def_start = .;
        KEEP(*(.image_def))
        _image_def_end = .;
        *(.text .text.*)
        *(.rodata .rodata.*)
        . = ALIGN(4);
        _etext = .;
    } > flash

    /* .data and .ramfunc are loaded from flash and copied to SRAM by Reset_Handler. */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.ramfunc .ramfunc.*)
        *(.data .data.*)
        . = ALIGN(4);
        _edata = .;
    } > sram AT > flash
    _sidata = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > sram

    /* Probe-only test storage; never initialised by startup. */
    .sram_lo_test   (NOLOAD) : { KEEP(*(.sram_lo_test)) }   > sram_lo_test
    .sram_hi_test   (NOLOAD) : { KEEP(*(.sram_hi_test)) }   > sram_hi_test
    .scratch_x_test (NOLOAD) : { KEEP(*(.scratch_x_test)) } > scratch_x_test
    .scratch_y_test (NOLOAD) : { KEEP(*(.scratch_y_test)) } > scratch_y_test

    /* Status block at 0x20081000, explicitly initialised by the probe. */
    .status (NOLOAD) :
    {
        _status_start = .;
        KEEP(*(.status))
        _status_end = .;
    } > scratch_y

    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.comment) }
}

/* ROM requirement: the IMAGE_DEF block loop must sit inside the first 4 KB (ds 5.2.7). */
ASSERT(_image_def_end - FLASH_ORIGIN <= 4096, "IMAGE_DEF block not inside the first 4 KB")
ASSERT(_image_def_end - _image_def_start == 20, "IMAGE_DEF block must be five words")

/* Image fits its flash region, including the .data load image. */
ASSERT(_sidata + (_edata - _sdata) <= FLASH_ORIGIN + FLASH_LENGTH, "image overflows its flash region")

/* .data/.ramfunc/.bss stay below the reserved stack. */
ASSERT(_ebss <= _stack_bottom, ".bss overlaps the reserved stack")
ASSERT(_stack_top == 0x2007fff8, "stack top must be 0x2007fff8")

/* Status block lives at the fixed address and clears the scratch_y test words. */
ASSERT(_status_start == 0x20081000, "status block must be at 0x20081000")
ASSERT(_status_end <= 0x20081ff8, "status block overlaps scratch_y test storage")
