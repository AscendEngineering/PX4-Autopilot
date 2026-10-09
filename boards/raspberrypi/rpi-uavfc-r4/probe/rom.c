/*
 * RP2350 boot ROM table lookup (ds 5.4, Table 453).
 */
#include "rom.h"

int rom_magic_ok(void)
{
	const volatile uint8_t *m = (const volatile uint8_t *)BOOTROM_MAGIC_ADDR;
	return m[0] == 'M' && m[1] == 'u' && m[2] == 0x02;
}

static rom_table_lookup_fn lookup_fn(void)
{
	uint16_t p = *(const volatile uint16_t *)BOOTROM_TABLE_LOOKUP_OFFSET;
	return (rom_table_lookup_fn)(uintptr_t)p;
}

void *rom_func_lookup(uint32_t code)
{
	if (!rom_magic_ok()) {
		return NULL;
	}

	return lookup_fn()(code, RT_FLAG_FUNC_ARM_SEC);
}

void *rom_data_lookup(uint32_t code)
{
	if (!rom_magic_ok()) {
		return NULL;
	}

	return lookup_fn()(code, RT_FLAG_DATA);
}
