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
 * @file rpi_flash_mtd.c
 * See rpi_flash_mtd.h.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>

#include <px4_arch/rpi_rom.h>
#include <px4_arch/rpi_flash.h>
#include <px4_arch/rpi_flash_mtd.h>

#define BLOCK_SIZE	RPI_FLASH_PAGE_SIZE
#define ERASE_SIZE	RPI_FLASH_SECTOR_SIZE
#define BLOCKS_PER_ERASE	(ERASE_SIZE / BLOCK_SIZE)

struct rpi_flash_mtd_s {
	struct mtd_dev_s mtd;	/* must be first: the MTD layer casts back */
	uint32_t offset;	/* first byte of the region in the part */
	size_t size;		/* bytes, whole sectors */
};

static bool in_region(const struct rpi_flash_mtd_s *priv, size_t start, size_t count)
{
	return start <= priv->size && count <= priv->size - start;
}

static int rpi_flash_mtd_erase(struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)dev;
	const size_t start = (size_t)startblock * ERASE_SIZE;
	const size_t count = nblocks * ERASE_SIZE;

	if (nblocks == 0) {
		return 0;
	}

	if (!in_region(priv, start, count)) {
		return -EINVAL;
	}

	if (rpi_flash_erase(priv->offset + start, count) != count) {
		return -EIO;
	}

	return (int)nblocks;
}

static ssize_t rpi_flash_mtd_bread(struct mtd_dev_s *dev, off_t startblock, size_t nblocks, uint8_t *buffer)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)dev;
	const size_t start = (size_t)startblock * BLOCK_SIZE;
	const size_t count = nblocks * BLOCK_SIZE;

	if (!in_region(priv, start, count)) {
		return -EINVAL;
	}

	memcpy(buffer, (const void *)(RPI_FLASH_BASE + priv->offset + start), count);
	return (ssize_t)nblocks;
}

static ssize_t rpi_flash_mtd_bwrite(struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
				    const uint8_t *buffer)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)dev;
	const size_t start = (size_t)startblock * BLOCK_SIZE;
	const size_t count = nblocks * BLOCK_SIZE;

	if (nblocks == 0) {
		return 0;
	}

	if (!in_region(priv, start, count)) {
		return -EINVAL;
	}

	if (rpi_flash_program(priv->offset + start, buffer, count) != count) {
		return -EIO;
	}

	return (ssize_t)nblocks;
}

static ssize_t rpi_flash_mtd_read(struct mtd_dev_s *dev, off_t offset, size_t nbytes, uint8_t *buffer)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)dev;

	if (!in_region(priv, (size_t)offset, nbytes)) {
		return -EINVAL;
	}

	memcpy(buffer, (const void *)(RPI_FLASH_BASE + priv->offset + (size_t)offset), nbytes);
	return (ssize_t)nbytes;
}

static int rpi_flash_mtd_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)dev;

	switch (cmd) {
	case MTDIOC_GEOMETRY: {
			struct mtd_geometry_s *geo = (struct mtd_geometry_s *)(uintptr_t)arg;

			if (geo == NULL) {
				return -EINVAL;
			}

			memset(geo, 0, sizeof(*geo));
			geo->blocksize = BLOCK_SIZE;
			geo->erasesize = ERASE_SIZE;
			geo->neraseblocks = priv->size / ERASE_SIZE;
			strlcpy(geo->model, "rp23xx-qspi", sizeof(geo->model));
			return 0;
		}

	case MTDIOC_BULKERASE:
		if (rpi_flash_erase(priv->offset, priv->size) != priv->size) {
			return -EIO;
		}

		return 0;

	case MTDIOC_ERASESTATE: {
			uint8_t *state = (uint8_t *)(uintptr_t)arg;

			if (state == NULL) {
				return -EINVAL;
			}

			*state = 0xff;
			return 0;
		}

	default:
		return -ENOTTY;
	}
}

struct mtd_dev_s *rpi_flash_mtd_initialize(uint32_t offset, size_t size)
{
	if (size == 0 || (offset % ERASE_SIZE) != 0 || (size % ERASE_SIZE) != 0 || offset + size < offset) {
		return NULL;
	}

	if (!rpi_flash_init()) {
		return NULL;
	}

	struct rpi_flash_mtd_s *priv = kmm_zalloc(sizeof(*priv));

	if (priv == NULL) {
		return NULL;
	}

	priv->mtd.erase = rpi_flash_mtd_erase;
	priv->mtd.bread = rpi_flash_mtd_bread;
	priv->mtd.bwrite = rpi_flash_mtd_bwrite;
	priv->mtd.read = rpi_flash_mtd_read;
	priv->mtd.ioctl = rpi_flash_mtd_ioctl;
	priv->mtd.name = "rpi_flash";
	priv->offset = offset;
	priv->size = size;

	return &priv->mtd;
}
