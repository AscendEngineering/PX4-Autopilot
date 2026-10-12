/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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

/**
 * @file board_reset.cpp
 *
 * Reset paths for RP2040/RP2350. The PX4 bootloader reads WATCHDOG
 * SCRATCH0 at start, which survives a SYSRESETREQ soft reset and not a
 * power-on; 0xb007b007 asks it to stay resident. The bootloader zeroes the
 * register as soon as it reads it, so a crash or watchdog reset never lands
 * there. REBOOT_TO_ISP hands the chip to the ROM's USB drive.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/shutdown.h>
#include <px4_arch/rpi_rom.h>
#include <errno.h>
#include <nuttx/board.h>
#include "arm_internal.h"

#if defined(CONFIG_BOARDCTL_RESET)

static constexpr uint32_t BOOT_TO_BOOTLOADER_MAGIC = 0xb007b007u;

int board_configure_reset(reset_mode_e mode, uint32_t arg)
{
	(void)arg;

	switch (mode) {
	case BOARD_RESET_MODE_CLEAR:
		putreg32(0, RPI_BOOT_SIGNATURE_REG);
		return OK;

	case BOARD_RESET_MODE_BOOT_TO_BL:
		putreg32(BOOT_TO_BOOTLOADER_MAGIC, RPI_BOOT_SIGNATURE_REG);
		return OK;

	default:
		return -EINVAL;
	}
}

int board_reset(int status)
{
	if (status == REBOOT_TO_BOOTLOADER) {
		board_configure_reset(BOARD_RESET_MODE_BOOT_TO_BL, 0);

	} else if (status == REBOOT_TO_ISP) {
		rpi_rom_reboot_bootsel();	/* returns only on failure; fall through to a plain reset */
	}

#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(status);
#endif

	up_systemreset();
	return 0;
}

#endif /* CONFIG_BOARDCTL_RESET */
