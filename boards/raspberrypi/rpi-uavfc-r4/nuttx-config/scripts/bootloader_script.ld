/****************************************************************************
 * boards/raspberrypi/rpi-uavfc-r4/nuttx-config/scripts/bootloader_script.ld
 *
 * PX4 bootloader image for the RP2350. Numbers repeat src/hw_config.h:
 * BOOTLOADER_RESERVATION_SIZE 128 KB at the XIP base, striped SRAM0-7.
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

MEMORY
{
	flash (rx)  : ORIGIN = 0x10000000, LENGTH = 128K   /* XIP, cached; sectors 0-31 */
	sram  (rwx) : ORIGIN = 0x20000000, LENGTH = 512K   /* SRAM0-7, striped */
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
/* Keep the production bootloader and flash path linked while status_main is
 * the NuttX entry point. This remains valid when phase 3 selects bootloader_main. */
EXTERN(bootloader_main)
ENTRY(_stext)

SECTIONS
{
	.text : {
		_stext = ABSOLUTE(.);
		KEEP(*(.vectors))

		/* Minimum "flash image boot" block, RP2350 datasheet 5.9.5. The ROM
		 * scans the first 4 KB for it; with no VECTOR_TABLE or ENTRY_POINT
		 * item it takes the vector table at the start of the image. */
		_image_def_start = ABSOLUTE(.);
		LONG(0xffffded3)	/* PICOBIN_BLOCK_MARKER_START */
		LONG(0x10210142)	/* IMAGE_TYPE: EXE, Arm, Secure, RP2350 */
		LONG(0x000001ff)	/* LAST item, size 1 */
		LONG(0x00000000)	/* link to self */
		LONG(0xab123579)	/* PICOBIN_BLOCK_MARKER_END */
		_image_def_end = ABSOLUTE(.);

		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);
	} > flash

	.init_section : {
		_sinit = ABSOLUTE(.);
		KEEP(*(.init_array .init_array.*))
		_einit = ABSOLUTE(.);
	} > flash

	.ARM.extab : {
		*(.ARM.extab*)
	} > flash

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > flash
	__exidx_end = ABSOLUTE(.);

	/* .ramfunc rides along with .data: rp23xx_start.c copies _eronly to
	 * _sdata.._edata before anything runs, which is how the ROM flash
	 * wrapper in bootloader/rpi/rpi_common/flash.c ends up in SRAM. */
	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.ramfunc .ramfunc.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		. = ALIGN(4);
		_edata = ABSOLUTE(.);
	} > sram AT > flash

	/* Where rp23xx_start.c copies .data from. Taken from the section's load
	 * address rather than from '.' before it: ld aligns the LMA to the
	 * section's alignment, and an 8-byte aligned initialised object would
	 * otherwise leave _eronly 4 bytes short of the real image. */
	_eronly = LOADADDR(.data);

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(8); /* NuttX's initial MSP is _ebss + idle stack size */
		_ebss = ABSOLUTE(.);
	} > sram

	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }
}

ASSERT(_image_def_end - _image_def_start == 20, "IMAGE_DEF block must be five words")
ASSERT(_image_def_end - ORIGIN(flash) <= 4096, "IMAGE_DEF block must end inside the first 4 KB")
ASSERT(_eronly == LOADADDR(.data), "_eronly must be the .data load address")
ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(flash) + LENGTH(flash), "bootloader image overflows its 128 KB reservation")
