/*
 * XIP-safe wrappers around the RP2350 ROM flash functions (ds 5.4.8.10).
 *
 * Sequence per erase or program:
 *   connect_internal_flash -> flash_exit_xip -> operation -> flash_flush_cache
 *   -> run the saved XIP setup function from an SRAM copy.
 *
 * While XIP is down any flash access bus-faults, so the wrapper runs from
 * .ramfunc with interrupts masked and calls nothing that lives in flash.
 * The XIP setup function image lives in boot RAM, which is not executable,
 * so it is copied to a stack buffer first (ds 5.2.6).
 */
#include "flash.h"
#include "rom.h"

#define BOOTRAM_BASE   0x400e0000u
#define BOOTRAM_END    0x400e0100u   /* first 256 bytes hold the saved XIP setup function */
#define XIP_SETUP_WORDS 64u

#define RAMFUNC __attribute__((section(".ramfunc"), noinline))

static const uint32_t *resolve_xip_setup(void)
{
	/* The 'X','F' data entry points at the saved function in boot RAM.
	 * Accept either a direct pointer into boot RAM or a pointer to one;
	 * fall back to the documented boot RAM base. */
	uintptr_t p = (uintptr_t)rom_data_lookup(ROM_DATA_SAVED_XIP_SETUP_FUNC_PTR);

	if (p >= BOOTRAM_BASE && p < BOOTRAM_END) {
		return (const uint32_t *)p;
	}

	if (p != 0 && (p & 3u) == 0) {
		uintptr_t q = *(const uintptr_t *)p;

		if (q >= BOOTRAM_BASE && q < BOOTRAM_END) {
			return (const uint32_t *)q;
		}
	}

	return (const uint32_t *)BOOTRAM_BASE;
}

int flash_rom_init(struct flash_rom *rom)
{
	rom->connect_internal_flash = rom_func_lookup(ROM_FUNC_CONNECT_INTERNAL_FLASH);
	rom->flash_exit_xip = rom_func_lookup(ROM_FUNC_FLASH_EXIT_XIP);
	rom->flash_range_erase = rom_func_lookup(ROM_FUNC_FLASH_RANGE_ERASE);
	rom->flash_range_program = rom_func_lookup(ROM_FUNC_FLASH_RANGE_PROGRAM);
	rom->flash_flush_cache = rom_func_lookup(ROM_FUNC_FLASH_FLUSH_CACHE);
	rom->xip_setup_func = resolve_xip_setup();

	if (!rom->connect_internal_flash || !rom->flash_exit_xip ||
	    !rom->flash_range_erase || !rom->flash_range_program ||
	    !rom->flash_flush_cache) {
		return -1;
	}

	return 0;
}

/* op: 0 = erase, 1 = program. Everything inside runs with XIP possibly down. */
RAMFUNC static void flash_op(const struct flash_rom *rom, int op,
			     uint32_t offset, const uint8_t *data, size_t count)
{
	uint32_t xip_setup[XIP_SETUP_WORDS];
	uint32_t primask;

	/* Copy the XIP setup image while flash is still readable (no memcpy: libc is in flash). */
	for (uint32_t i = 0; i < XIP_SETUP_WORDS; i++) {
		xip_setup[i] = rom->xip_setup_func[i];
	}

	__asm volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask) :: "memory");

	rom->connect_internal_flash();
	rom->flash_exit_xip();

	if (op == 0) {
		rom->flash_range_erase(offset, count, 4096u, 0x20u);

	} else {
		rom->flash_range_program(offset, data, count);
	}

	rom->flash_flush_cache();

	/* Restore the ROM-discovered XIP read mode from the SRAM copy (Thumb bit set). */
	((void (*)(void))((uintptr_t)xip_setup | 1u))();

	__asm volatile("dsb sy\n\tisb sy" ::: "memory");
	__asm volatile("msr primask, %0" :: "r"(primask) : "memory");
}

void flash_erase(const struct flash_rom *rom, uint32_t offset, size_t count)
{
	flash_op(rom, 0, offset, NULL, count);
}

void flash_program(const struct flash_rom *rom, uint32_t offset, const uint8_t *data, size_t count)
{
	flash_op(rom, 1, offset, data, count);
}
