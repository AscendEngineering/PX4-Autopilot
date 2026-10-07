/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * @file board_mcu_version.c
 * Implementation of RP2040/RP2350 based SoC version API
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

// SYSINFO CHIP_ID identifies the chip and its silicon revision (the ARM CPUID
// register would only identify the core). Layout, same on both chips:
//   MANUFACTURER [11:0] = 0x927   PART [27:12] = RPI_CHIP_ID_PART   REVISION [31:28]
#define CHIP_ID				getreg32(RPI_SYSINFO_BASE + 0x0)
#define CHIP_ID_MANUFACTURER_MASK	0x00000fff
#define CHIP_ID_MANUFACTURER_RPI	0x927
#define CHIP_ID_PART_SHIFT		12
#define CHIP_ID_PART_MASK		0xffff
#define CHIP_ID_REVISION_SHIFT		28
#define CHIP_ID_REVISION_MASK		0xf

int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	const uint32_t chip_id = CHIP_ID;
	const int revision = (chip_id >> CHIP_ID_REVISION_SHIFT) & CHIP_ID_REVISION_MASK;

	if ((chip_id & CHIP_ID_MANUFACTURER_MASK) != CHIP_ID_MANUFACTURER_RPI ||
	    ((chip_id >> CHIP_ID_PART_SHIFT) & CHIP_ID_PART_MASK) != RPI_CHIP_ID_PART) {
		return -1;
	}

	if (revstr) {
		*revstr = RPI_CHIP_NAME;
	}

	if (rev) {
		// Raw silicon revision number as the chip reports it (RP2040: 1 = B0/B1, 2 = B2)
		*rev = revision < 10 ? '0' + revision : '?';
	}

	if (errata) {
		// None known that PX4 needs to warn about. (RP2040's missing unique id is
		// not silicon errata; see board_identity.c.)
		*errata = NULL;
	}

	return revision;
}
