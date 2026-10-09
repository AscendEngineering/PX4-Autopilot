/*
 * RP2350 boot ROM table lookup (ds 5.4).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define ROM_TABLE_CODE(c1, c2)  ((uint32_t)(c1) | ((uint32_t)(c2) << 8))

#define ROM_FUNC_CONNECT_INTERNAL_FLASH   ROM_TABLE_CODE('I', 'F')
#define ROM_FUNC_FLASH_EXIT_XIP           ROM_TABLE_CODE('E', 'X')
#define ROM_FUNC_FLASH_RANGE_ERASE        ROM_TABLE_CODE('R', 'E')
#define ROM_FUNC_FLASH_RANGE_PROGRAM      ROM_TABLE_CODE('R', 'P')
#define ROM_FUNC_FLASH_FLUSH_CACHE        ROM_TABLE_CODE('F', 'C')
#define ROM_FUNC_GET_SYS_INFO             ROM_TABLE_CODE('G', 'S')
#define ROM_DATA_SAVED_XIP_SETUP_FUNC_PTR ROM_TABLE_CODE('X', 'F')

/* Lookup flags (pico-sdk bootrom_constants.h; ds 5.4.1 code sample). */
#define RT_FLAG_FUNC_ARM_SEC    0x0004u
#define RT_FLAG_DATA            0x0040u

/* Well-known ROM locations (ds Table 453). */
#define BOOTROM_MAGIC_ADDR          0x00000010u
#define BOOTROM_TABLE_LOOKUP_OFFSET 0x00000016u

#define SYS_INFO_CHIP_INFO      0x0001u

typedef void *(*rom_table_lookup_fn)(uint32_t code, uint32_t mask);
typedef void (*rom_void_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t addr, size_t count, uint32_t block_size, uint8_t block_cmd);
typedef void (*rom_flash_range_program_fn)(uint32_t addr, const uint8_t *data, size_t count);
typedef int (*rom_get_sys_info_fn)(uint32_t *out, uint32_t out_words, uint32_t flags);

/* Returns 1 if the ROM magic 'M','u',0x02 is present at 0x10. */
int rom_magic_ok(void);

/* Function lookup for Secure Arm code; NULL if not found. */
void *rom_func_lookup(uint32_t code);

/* Data lookup; NULL if not found. */
void *rom_data_lookup(uint32_t code);
