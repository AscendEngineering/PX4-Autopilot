/****************************************************************************
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

/**
 * @file mtd.cpp
 * Parameter storage of the RPI-UAVFC-R4: the 64 KB that the bootloader keeps
 * free at the top of the 4 MB QSPI flash (hw_config.h APP_RESERVATION_SIZE),
 * served as one MTD device by the shared ROM flash library. The common MTD
 * code puts an FTL and a character device on it, so the parameters are the
 * file /fs/nor/mtd_params, as on the FRAM boards, and "param export", the SD
 * card backup and the mtd tools all work on it.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/px4_mtd.h>
#include <px4_arch/rpi_rom.h>
#include <px4_arch/rpi_flash.h>
#include <px4_arch/rpi_flash_mtd.h>
#include "board_config.h"

static const px4_mft_device_t qspi_flash = {
	.bus_type = px4_mft_device_t::ONCHIP,
	.devid    = 0
};

/* One partition over the whole region, in 256-byte program pages. */
static const px4_mtd_entry_t qspi_params = {
	.device = &qspi_flash,
	.npart = 1,
	.partd = {
		{
			.type = MTD_PARAMETERS,
			.path = "/fs/nor/mtd_params",
			.nblocks = BOARD_PARAMS_FLASH_SIZE / RPI_FLASH_PAGE_SIZE
		}
	},
};

static const px4_mtd_manifest_t board_mtd_config = {
	.nconfigs   = 1,
	.entries = {
		&qspi_params
	}
};

static const px4_mft_entry_s mtd_mft = {
	.type = MTD,
	.pmft = (void *) &board_mtd_config,
};

static const px4_mft_s mft = {
	.nmft = 1,
	.mfts = {
		&mtd_mft
	}
};

const px4_mft_s *board_get_manifest(void)
{
	return &mft;
}

struct mtd_dev_s *board_mtd_onchip_initialize(uint32_t devid)
{
	return rpi_flash_mtd_initialize(BOARD_PARAMS_FLASH_ADDRESS - RPI_FLASH_BASE, BOARD_PARAMS_FLASH_SIZE);
}
