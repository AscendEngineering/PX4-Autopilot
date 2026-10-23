#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/px4_mtd.h>
#include <px4_arch/rpi_flash.h>
#include "board_config.h"

#include <nuttx/mtd/mtd.h>
#include <string.h>

#define ERASE_SIZE RPI_FLASH_SECTOR_SIZE
#define BLOCK_SIZE RPI_FLASH_PAGE_SIZE
struct rpi_flash_mtd_s {
	struct mtd_dev_s mtd;
	uint32_t offset;
	size_t size;
};
/**
 * RP2350 QSPI Flash MTD Driver Bridge
 *
 * This driver wraps the rpi_flash_* ROM helpers to provide a standard
 * NuttX MTD interface for the board's external QSPI flash.
 */
static bool in_region(const struct rpi_flash_mtd_s *priv, size_t start, size_t count)
{
	return start <= priv->size && count <= priv->size - start;
}

static int rpi_flash_mtd_erase(struct mtd_dev_s *mtd, off_t startblock, size_t nblocks)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)mtd;
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

static ssize_t rpi_flash_mtd_bread(struct mtd_dev_s *mtd, off_t startblock,
			      size_t nblocks, uint8_t *buffer)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)mtd;
	const size_t start = (size_t)startblock * BLOCK_SIZE;
	const size_t count = nblocks * BLOCK_SIZE;

	if (!in_region(priv, start, count)) {
		return -EINVAL;
	}

	memcpy(buffer, (const void *)(RPI_FLASH_BASE + priv->offset + start), count);
	return (ssize_t)nblocks;
}

static ssize_t rpi_flash_mtd_bwrite(struct mtd_dev_s *mtd, off_t startblock,
			       size_t nblocks, const uint8_t *buffer)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)mtd;
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

static ssize_t rpi_flash_mtd_read(struct mtd_dev_s *mtd, off_t offset, size_t nbytes, uint8_t *buffer)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)mtd;

	if (!in_region(priv, (size_t)offset, nbytes)) {
		return -EINVAL;
	}

	memcpy(buffer, (const void *)(RPI_FLASH_BASE + priv->offset + (size_t)offset), nbytes);
	return (ssize_t)nbytes;

}

static int rpi_flash_mtd_ioctl(struct mtd_dev_s *mtd, int cmd, unsigned long arg)
{
	struct rpi_flash_mtd_s *priv = (struct rpi_flash_mtd_s *)mtd;

	switch (cmd) {
	case MTDIOC_GEOMETRY: {
			struct mtd_geometry_s *geo = (struct mtd_geometry_s *)(uintptr_t)arg;

			if (geo == NULL) {
				return -EINVAL;
			}

			memset(geo, 0, sizeof(*geo));
			geo->blocksize = RPI_FLASH_PAGE_SIZE;
			geo->erasesize = ERASE_SIZE;
			geo->neraseblocks = priv->size / ERASE_SIZE;
			strlcpy(geo->model, "rp23xx-qspi", sizeof(geo->model));
			return 0;
		}

	case MTDIOC_BULKERASE:
		if (rpi_flash_erase(priv->offset, priv->size) != BOARD_PARAMS_FLASH_SIZE) {
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


// Physical bus devices
const px4_mft_device_t qspi_device = {
	.bus_type = px4_mft_device_t::RP23XXQSPI,
	.devid = 0
};

// MTD partition entries
const px4_mtd_entry_t rpi_params_mtd = {
	.device = &qspi_device,
	.npart = 1,
	.partd = {
		{
			.type = MTD_PARAMETERS,
			.path = "/fs/nor/mtd_params",
			.nblocks = BOARD_PARAMS_FLASH_SIZE / RPI_FLASH_PAGE_SIZE,
		}
	},
};

// MTD manifests
static const px4_mtd_manifest_t mft = {
	.nconfigs = 1,
	.entries = {
		&rpi_params_mtd,
	}
};

// MFT entries
static const px4_mft_entry_s mtd_mft_qspi = {
	.type = MTD,
	.pmft = (void *) &mft,
};

// Final manifset for QSPI flash
static const px4_mft_s mft_qspi = {
	.nmft = 1,
	.mfts = {&mtd_mft_qspi}
};

static const px4_mft_s *g_manifest = &mft_qspi;

const px4_mft_s *board_get_manifest(void)
{
	return g_manifest;
}


struct mtd_dev_s* rpi_flash_mtd_initialize(uint32_t offset, size_t size)
{
	if (size == 0 || (offset % ERASE_SIZE) != 0 || (size % ERASE_SIZE) != 0 || offset + size < offset) {
		return NULL;
	}

	if (!rpi_flash_init()) {
		return NULL;
	}

	struct rpi_flash_mtd_s *priv = (rpi_flash_mtd_s*)kmm_zalloc(sizeof(*priv));

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

struct mtd_dev_s *board_mtd_rp23xx_initialize(uint32_t devid)
{
	return rpi_flash_mtd_initialize(BOARD_PARAMS_FLASH_ADDRESS - RPI_FLASH_BASE, BOARD_PARAMS_FLASH_SIZE);
}
