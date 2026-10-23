#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/px4_mtd.h>
#include <px4_arch/rpi_flash.h>
#include "board_config.h"

#include <nuttx/mtd/mtd.h>
#include <string.h>

/**
 * RP2350 QSPI Flash MTD Driver Bridge
 *
 * This driver wraps the rpi_flash_* ROM helpers to provide a standard
 * NuttX MTD interface for the board's external QSPI flash.
 */

static int rpi_qspi_read(struct mtd_dev_s *mtd, off_t offset, size_t len, uint8_t *buf)
{
	/* QSPI flash is mapped into the XIP window starting at RPI_FLASH_BASE */
	const uint8_t *xip_ptr = (const uint8_t *)(RPI_FLASH_BASE + offset);
	memcpy(buf, xip_ptr, len);
	return 0;
}

static int rpi_qspi_erase(struct mtd_dev_s *mtd, off_t offset, size_t len)
{
	if (rpi_flash_erase(offset, len) != len) {
		return -EIO;
	}

	return 0;
}

static int rpi_qspi_write(struct mtd_dev_s *mtd, off_t offset, size_t len, const uint8_t *buf)
{
	if (rpi_flash_program(offset, buf, len) != len) {
		return -EIO;
	}

	return 0;
}

static ssize_t rpi_qspi_bread(struct mtd_dev_s *mtd, off_t startblock,
			      size_t nblocks, uint8_t *buffer)
{
	/*
	 * NuttX MTD bread expects offset in blocks.
	 * Since we treat this as byte-addressable (blocksize=1),
	 * startblock is the byte offset.
	 */
	return rpi_qspi_read(mtd, (uint32_t)startblock, (size_t)nblocks, buffer);
}

static ssize_t rpi_qspi_bwrite(struct mtd_dev_s *mtd, off_t startblock,
			       size_t nblocks, const uint8_t *buffer)
{
	return rpi_qspi_write(mtd, (uint32_t)startblock, (size_t)nblocks, buffer);
}

static int rpi_flash_mtd_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
	// struct mtd_dev_s *priv = (struct mtd_dev_s *)dev;

	switch (cmd) {
	case MTDIOC_GEOMETRY: {
			struct mtd_geometry_s *geo = (struct mtd_geometry_s *)(uintptr_t)arg;

			if (geo == NULL) {
				return -EINVAL;
			}

			memset(geo, 0, sizeof(*geo));
			geo->blocksize = RPI_FLASH_PAGE_SIZE;
			geo->erasesize = RPI_FLASH_SECTOR_SIZE;
			geo->neraseblocks = BOARD_PARAMS_FLASH_SIZE / RPI_FLASH_SECTOR_SIZE;
			strlcpy(geo->model, "rp23xx-qspi", sizeof(geo->model));
			return 0;
		}

	case MTDIOC_BULKERASE:
		// if (rpi_flash_erase(priv->offset, ) != BOARDS_PARAM_FLASH_SIZE) {
		// 	return -EIO;
		// }

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

/*
 * Note: struct mtd_dev_s in NuttX does not have a 'blocksize' member.
 * Geometry is handled via MTDIOC_GEOMETRY ioctl.
 * We define the structure without the invalid member.
 */
static struct mtd_dev_s rpi_qspi_dev = {
	.erase = rpi_qspi_erase,
	.bread = rpi_qspi_bread,
	.bwrite = rpi_qspi_bwrite,
	.read = rpi_qspi_read,
	.write = rpi_qspi_write,
	.ioctl = rpi_flash_mtd_ioctl,
	.name = "rpi_qspi",
};

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
			.nblocks = BOARD_PARAMS_FLASH_SIZE / RPI_FLASH_SECTOR_SIZE,
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

extern "C" {

	int rpi_qspi_storage_initialize(void)
	{
		// This might already attach
		struct mtd_dev_s *mtd_dev = &rpi_qspi_dev;
		int ret = -ENODEV;

		ret = register_mtddriver("/fs/nor", mtd_dev, 0755, NULL);

		if (ret < 0) {
			syslog(LOG_ERR, "ERROR: failed to register mtd driver: %d\n", ret);
		}

		syslog(LOG_INFO, "[boot] W25N MTD registered at /fs/nor\n");


		return ret;
	}
	int rpi_qspi_attach(mtd_instance_s *instance)
	{
		int rv = rpi_qspi_storage_initialize();

		if (rv != OK) {
			syslog(LOG_ERR, "failed to initalize rpi qspi flash");
			return -ENXIO;
		}

		instance->mtd_dev = &rpi_qspi_dev;
		return OK;
	}

} // extern "C"
