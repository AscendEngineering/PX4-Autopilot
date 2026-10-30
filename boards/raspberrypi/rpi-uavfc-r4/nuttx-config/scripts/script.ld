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
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS, SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

MEMORY
{
	flash (rx)  : ORIGIN = 0x10020000, LENGTH = 3904K
	sram  (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
	sram8 (rwx) : ORIGIN = 0x20080000, LENGTH = 4K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)
EXTERN(board_get_manifest)
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

	/* .ramfunc rides along with .data so the ROM flash wrapper in
	 * px4/rpi/rpi_common/flash/rpi_flash.c runs from SRAM (flash-based
	 * parameters). rp23xx_start.c copies _eronly to _sdata.._edata first. */
	.data : {
	_sdata = ABSOLUTE(.);
	*(.data .data.*)
	. = ALIGN(4);
	_edata = ABSOLUTE(.);
	} > sram AT > flash
	_eronly = LOADADDR(.data);

	/* Hot functions copied to SRAM at boot */
	.ramfunc : {
	_sramfunc = ABSOLUTE(.);
	*(.ramfunc .ramfunc.*)
		*(.text._ZN12EKF2Selector17UpdateErrorScoresEv)
		*(.ramfunc._ZN12EKF2Selector17UpdateErrorScoresEv)
		*(.text._ZN12EKF2Selector3RunEv)
		*(.ramfunc._ZN12EKF2Selector3RunEv)
		*(.text._ZN12EKF2SelectorC1Ev)
		*(.ramfunc._ZN12EKF2SelectorC1Ev)
		*(.text._ZN12EKF2SelectorC2Ev)
		*(.ramfunc._ZN12EKF2SelectorC2Ev)
		*(.text._ZN3Ekf16controlMagFusionERKN9estimator9imuSampleE)
		*(.ramfunc._ZN3Ekf16controlMagFusionERKN9estimator9imuSampleE)
		*(.text._ZN3Ekf18controlEvPosFusionERKN9estimator9imuSampleERKNS0_15extVisionSampleEbbbR24estimator_aid_source2d_s)
		*(.ramfunc._ZN3Ekf18controlEvPosFusionERKN9estimator9imuSampleERKNS0_15extVisionSampleEbbbR24estimator_aid_source2d_s)
		*(.text._ZN3Ekf20controlGravityFusionERKN9estimator9imuSampleE)
		*(.ramfunc._ZN3Ekf20controlGravityFusionERKN9estimator9imuSampleE)
		*(.text._ZN3Ekf21controlEvHeightFusionERKN9estimator9imuSampleERKNS0_15extVisionSampleEbbbR24estimator_aid_source1d_s)
		*(.ramfunc._ZN3Ekf21controlEvHeightFusionERKN9estimator9imuSampleERKNS0_15extVisionSampleEbbbR24estimator_aid_source1d_s)
		*(.text._ZN3Ekf22controlRangeHaglFusionERKN9estimator9imuSampleE)
		*(.ramfunc._ZN3Ekf22controlRangeHaglFusionERKN9estimator9imuSampleE)
		*(.text._ZN3Ekf23controlBaroHeightFusionERKN9estimator9imuSampleE)
		*(.ramfunc._ZN3Ekf23controlBaroHeightFusionERKN9estimator9imuSampleE)
		*(.text._ZN3Ekf24controlOpticalFlowFusionERKN9estimator9imuSampleE)
		*(.ramfunc._ZN3Ekf24controlOpticalFlowFusionERKN9estimator9imuSampleE)
		*(.text._ZN3Ekf27controlExternalVisionFusionERKN9estimator9imuSampleE)
		*(.ramfunc._ZN3Ekf27controlExternalVisionFusionERKN9estimator9imuSampleE)
		*(.text._ZN4EKF216updateParamsImplEv)
		*(.ramfunc._ZN4EKF216updateParamsImplEv)
		*(.text._ZN4EKF220PublishLocalPositionERKy)
		*(.ramfunc._ZN4EKF220PublishLocalPositionERKy)
		*(.text._ZN4EKF222PublishAidSourceStatusERKy)
		*(.ramfunc._ZN4EKF222PublishAidSourceStatusERKy)
		*(.text._ZN4EKF23RunEv)
		*(.ramfunc._ZN4EKF23RunEv)
		*(.text._ZN4EKF2C1EbRKN3px411wq_config_tEb)
		*(.ramfunc._ZN4EKF2C1EbRKN3px411wq_config_tEb)
		*(.text._ZN4EKF2C2EbRKN3px411wq_config_tEb)
		*(.ramfunc._ZN4EKF2C2EbRKN3px411wq_config_tEb)
		*(.text._ZN7sensors10VehicleIMU7PublishEv)
		*(.ramfunc._ZN7sensors10VehicleIMU7PublishEv)
		*(.text._ZN7sensors14VehicleAirData3RunEv)
		*(.ramfunc._ZN7sensors14VehicleAirData3RunEv)
		*(.text._ZN7sensors18VehicleOpticalFlow3RunEv)
		*(.ramfunc._ZN7sensors18VehicleOpticalFlow3RunEv)
		*(.text._ZN7sensors19VehicleMagnetometer3RunEv)
		*(.ramfunc._ZN7sensors19VehicleMagnetometer3RunEv)
		*(.text._ZN7sensors22VehicleAngularVelocity16ParametersUpdateEb)
		*(.ramfunc._ZN7sensors22VehicleAngularVelocity16ParametersUpdateEb)
		*(.text._ZN9Navigator3runEv)
		*(.ramfunc._ZN9Navigator3runEv)
		*(.text._ZN9NavigatorC1Ev)
		*(.ramfunc._ZN9NavigatorC1Ev)
		*(.text._ZN9NavigatorC2Ev)
		*(.ramfunc._ZN9NavigatorC2Ev)
		*(.text._ZNK15AttitudeControl6updateERKN6matrix10QuaternionIfEE)
		*(.ramfunc._ZNK15AttitudeControl6updateERKN6matrix10QuaternionIfEE)
	. = ALIGN(4);
	_eramfunc = ABSOLUTE(.);
	} > sram AT > flash
	_lramfunc = LOADADDR(.ramfunc);

	/* Critical functions in SRAM8 (Scratch X) for zero-contention I-CODE fetch */
	.scratchx : {
	_sscratchx = ABSOLUTE(.);
		*(.scratchx)
		*(.text._ZN22MulticopterRateControl3RunEv.part.0)
		*(.ramfunc._ZN22MulticopterRateControl3RunEv.part.0)
		*(.text._ZN26MulticopterAttitudeControl3RunEv.part.0)
		*(.ramfunc._ZN26MulticopterAttitudeControl3RunEv.part.0)
		*(.text._ZN3Ekf12predictStateERKN9estimator9imuSampleE)
		*(.ramfunc._ZN3Ekf12predictStateERKN9estimator9imuSampleE)
		*(.text._ZN3Ekf17predictCovarianceERKN9estimator9imuSampleE)
		*(.ramfunc._ZN3Ekf17predictCovarianceERKN9estimator9imuSampleE)
		*(.text._ZN7sensors10VehicleIMU10UpdateGyroEv)
		*(.ramfunc._ZN7sensors10VehicleIMU10UpdateGyroEv)
		*(.text._ZN7sensors10VehicleIMU11UpdateAccelEv)
		*(.ramfunc._ZN7sensors10VehicleIMU11UpdateAccelEv)
	. = ALIGN(4);
	_escratchx = ABSOLUTE(.);
	} > sram8 AT > flash
	_lscratchx = LOADADDR(.scratchx);

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

	ASSERT(_image_def_end - _image_def_start == 20, "IMAGE_DEF block must be five words")
	ASSERT(_image_def_end - ORIGIN(flash) <= 4096, "IMAGE_DEF block must end inside the first 4 KB")
	ASSERT(_eronly == LOADADDR(.data), "_eronly must be LOADADDR(.data)")
	ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(flash) + LENGTH(flash), "application image overflows its 3904 KB window")
}
