#!/usr/bin/env python3
"""Host checks for the rpi bootloader sources (flash.c, main.c, systick.c).

Run with: python3 platforms/nuttx/src/bootloader/rpi/rpi_common/tests/test_bootloader.py
Requires Linux and cc; maps fake peripheral pages and a fake 4 MB XIP window
in the test process. The NuttX register headers are the real ones from the
submodule, the boot ROM is a fake function table, the GPIO API is stubbed and
the common flash cache is compiled from its real source. Nothing here runs a
ROM flash command or a jump; that stays with the bench checklist in the spec.
"""
import pathlib
import subprocess
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
RPI_COMMON = HERE.parent
BOOTLOADER = RPI_COMMON.parents[1]          # platforms/nuttx/src/bootloader
ROOT = next(p for p in HERE.parents if (p / "Tools").is_dir())
NUTTX_ARCH = ROOT / "platforms/nuttx/NuttX/nuttx/arch/arm/src"

# Values the board's hw_config.h is expected to carry (design spec 3.1, 4.4)
HW_CONFIG = """
#pragma once
#define USB0_DEV 0x01
#define BOARD_TYPE 7300
#define BOOTLOADER_RESERVATION_SIZE (128 * 1024)
#define APP_LOAD_ADDRESS 0x10020000
#define APP_RESERVATION_SIZE (64 * 1024)
#define BOARD_FLASH_SIZE (4 * 1024 * 1024)
#define BOARD_FLASH_SECTORS 1024
#define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 32
#define BOOTLOADER_DELAY 5000
#define INTERFACE_USB 1
#define INTERFACE_USB_CONFIG "/dev/ttyACM0"
#define INTERFACE_USART 0
#define SERIAL_BREAK_DETECT_DISABLED 1
#define BOARD_PIN_LED_ACTIVITY 0
#define BOARD_PIN_LED_BOOTLOADER 1
#define BOARD_LED_ON 0
#define BOARD_LED_OFF 1
#define ARCH_SN_MAX_LENGTH 8
#define BOOT_DEVICES_SELECTION USB0_DEV
#define BOOT_DEVICES_FILTER_ONUSB USB0_DEV
#if defined(TEST_VBUS)
#define BOARD_VBUS 24
#endif
"""

STUBS = {
    "nuttx/config.h": "#pragma once\n#define CONFIG_ARCH_CHIP_RP23XX 1\n",
    "chip.h": "",
    "nuttx/progmem.h": "",
    "arm_internal.h": """
#pragma once
#include <stdint.h>
#define getreg32(a)   (*(volatile uint32_t *)(uintptr_t)(a))
#define putreg32(v,a) (*(volatile uint32_t *)(uintptr_t)(a) = (v))
static inline void modifyreg32(uintptr_t a, uint32_t clear, uint32_t set)
{ putreg32((getreg32(a) & ~clear) | set, a); }
""",
    "arch/board/board.h": "#pragma once\n#define BOARD_SYS_FREQ 150000000\n",
    "rp23xx_gpio.h": """
#pragma once
#include <stdint.h>
#include <stdbool.h>
void rp23xx_gpio_init(uint32_t gpio);
void rp23xx_gpio_put(uint32_t gpio, int set);
bool rp23xx_gpio_get(uint32_t gpio);
void rp23xx_gpio_setdir(uint32_t gpio, int out);
""",
    "hw_config.h": HW_CONFIG,
}

TEST_C = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include "hw_config.h"
#include "bl.h"
#include "bl_chip.h"
#include <lib/systick.h>
#include <lib/flash_cache.h>
#include <nvic.h>

#define XIP ((volatile uint8_t *)(uintptr_t)BL_FLASH_BASE)

/* declared nowhere in bl.h: provided by flash.c and main.c */
ssize_t arch_flash_write(uintptr_t address, const void *buffer, size_t buflen);
int bootloader_main(int argc, char *argv[]);

/* ---- fake peripherals ------------------------------------------------- */

static void map(uintptr_t base, size_t len)
{
	void *want = (void *)(base & ~(uintptr_t)4095);
	void *got = mmap(want, len, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	if (got != want) { perror("mmap"); exit(2); }
}

/* GPIO stubs: function/direction/level per pin */
static int gpio_sio[64], gpio_out[64], gpio_level[64];
void rp23xx_gpio_init(uint32_t g) { gpio_sio[g] = 1; gpio_out[g] = 0; }   /* level: what the pin reads */
void rp23xx_gpio_put(uint32_t g, int set) { gpio_level[g] = set; }
bool rp23xx_gpio_get(uint32_t g) { return gpio_level[g]; }
void rp23xx_gpio_setdir(uint32_t g, int out) { gpio_out[g] = out; }

/* ROM fakes: record the call sequence and act on the XIP window */
static char calls[64];          /* sequence of the most recent calls, for order checks */
static unsigned ncalls;         /* every call, unbounded */
static uint32_t last_erase_addr, last_erase_count, last_erase_block; static uint8_t last_erase_cmd;
static uint32_t last_prog_addr, last_prog_count;
static uint32_t xip_image[64];          /* what the "saved XIP setup function" contains */
static uintptr_t xip_setup_ptr_holder;  /* ROM data entry points here, which points into boot RAM */
static int restore_ok;

static void log_call(char c) { if (ncalls < sizeof(calls) - 1) { calls[ncalls] = c; calls[ncalls + 1] = 0; } ncalls++; }
static void fake_connect(void) { log_call('C'); }
static void fake_exit_xip(void) { log_call('X'); }
static void fake_flush(void) { log_call('F'); }
static void fake_erase(uint32_t addr, size_t count, uint32_t block, uint8_t cmd)
{
	log_call('E'); last_erase_addr = addr; last_erase_count = count; last_erase_block = block; last_erase_cmd = cmd;
	assert(addr % 4096 == 0 && count % 4096 == 0);
	memset((void *)(XIP + addr), 0xff, count);
}
static void fake_program(uint32_t addr, const uint8_t *data, size_t count)
{
	log_call('P'); last_prog_addr = addr; last_prog_count = count;
	assert(addr % 256 == 0 && count % 256 == 0);
	for (size_t i = 0; i < count; i++) XIP[addr + i] &= data[i];   /* NOR: only clears bits */
}
static void *fake_lookup(uint32_t code, uint32_t mask)
{
	switch (code) {
	case 'I' | 'F' << 8: return mask == BL_ROM_RT_FLAG_FUNC_ARM_SEC ? (void *)fake_connect : NULL;
	case 'E' | 'X' << 8: return mask == BL_ROM_RT_FLAG_FUNC_ARM_SEC ? (void *)fake_exit_xip : NULL;
	case 'R' | 'E' << 8: return mask == BL_ROM_RT_FLAG_FUNC_ARM_SEC ? (void *)fake_erase : NULL;
	case 'R' | 'P' << 8: return mask == BL_ROM_RT_FLAG_FUNC_ARM_SEC ? (void *)fake_program : NULL;
	case 'F' | 'C' << 8: return mask == BL_ROM_RT_FLAG_FUNC_ARM_SEC ? (void *)fake_flush : NULL;
	case 'X' | 'F' << 8: return mask == BL_ROM_RT_FLAG_DATA ? (void *)&xip_setup_ptr_holder : NULL;
	}
	return NULL;
}
static bool rom_present = true;
bool bl_rom_present(void) { return rom_present; }
bl_rom_table_lookup_fn bl_rom_table_lookup(void) { return fake_lookup; }
void bl_xip_restore(const uint32_t *copy)
{
	log_call('R');
	restore_ok = memcmp(copy, xip_image, sizeof(xip_image)) == 0;
	assert((uintptr_t)copy < BL_FLASH_BASE || (uintptr_t)copy >= BL_FLASH_BASE + BOARD_FLASH_SIZE); /* copy is not in flash */
}

/* common bootloader fakes */
static int jumped, cinit_calls;
void jump_to_app(void) { jumped++; }
void cinit(void *config, uint8_t interface) { (void)config; (void)interface; cinit_calls++; }
void bootloader(unsigned timeout)
{
	printf("bootloader timeout=%u scratch=%#x jumped=%d cinit=%d\n", timeout,
	       (unsigned)getreg32(BL_BOOT_SIGNATURE_REG), jumped, cinit_calls);
	exit(0);
}

/* ---- checks ----------------------------------------------------------- */

static void reset_log(void) { ncalls = 0; calls[0] = 0; restore_ok = 0; }

static void test_flash(void)
{
	const unsigned app_end = (BOARD_FLASH_SIZE - APP_RESERVATION_SIZE) / 4096;   /* 1008 */
	assert(flash_func_sector_size(0) == 4096 && flash_func_sector_size(BOARD_FLASH_SECTORS - 1) == 4096);
	assert(flash_func_sector_size(BOARD_FLASH_SECTORS) == 0);

	memset((void *)XIP, 0xa5, BOARD_FLASH_SIZE);   /* everything written */

	/* bootloader sectors are never erased, with or without force */
	for (unsigned s = 0; s < BOARD_FIRST_FLASH_SECTOR_TO_ERASE; s++) { flash_func_erase_sector(s, false); flash_func_erase_sector(s, true); }
	assert(ncalls == 0 && XIP[0] == 0xa5 && XIP[BOARD_FIRST_FLASH_SECTOR_TO_ERASE * 4096 - 1] == 0xa5);

	/* params reservation: untouched normally, erased on a full erase */
	flash_func_erase_sector(app_end, false);
	assert(ncalls == 0 && XIP[app_end * 4096] == 0xa5);
	flash_func_erase_sector(app_end, true);
	assert(strcmp(calls, "CXEFR") == 0 && restore_ok);
	assert(last_erase_addr == app_end * 4096 && last_erase_count == 65536 && last_erase_block == 65536 && last_erase_cmd == 0xd8);
	assert(XIP[app_end * 4096] == 0xff && XIP[BOARD_FLASH_SIZE - 1] == 0xff && XIP[app_end * 4096 - 1] == 0xa5);
	reset_log();

	/* the application: walk every sector as the protocol does */
	for (unsigned s = 0; flash_func_sector_size(s) != 0; s++) flash_func_erase_sector(s, false);
	assert(ncalls == 5 * (app_end - BOARD_FIRST_FLASH_SECTOR_TO_ERASE) / 16);   /* one block erase per 64 kB */
	assert(last_erase_cmd == 0xd8 && last_erase_addr == (app_end - 16) * 4096);
	assert(XIP[BOARD_FIRST_FLASH_SECTOR_TO_ERASE * 4096] == 0xff && XIP[app_end * 4096 - 1] == 0xff);
	assert(XIP[BOARD_FIRST_FLASH_SECTOR_TO_ERASE * 4096 - 1] == 0xa5);
	reset_log();

	/* blank sectors cost nothing */
	for (unsigned s = 0; flash_func_sector_size(s) != 0; s++) flash_func_erase_sector(s, false);
	assert(ncalls == 0);

	/* a single dirty sector that is not block aligned gets a 4 kB erase */
	XIP[1007 * 4096 + 100] = 0;
	flash_func_erase_sector(1007, false);
	assert(strcmp(calls, "CXEFR") == 0 && last_erase_addr == 1007 * 4096 && last_erase_count == 4096 && last_erase_block == 4096 && last_erase_cmd == 0x20);
	reset_log();

	/* out of range */
	flash_func_erase_sector(BOARD_FLASH_SECTORS, true);
	flash_func_erase_sector(100000, true);
	assert(ncalls == 0);

	/* page writes through the common flash cache: 64 words per page */
	assert(FC_NUMBER_WORDS == 64);
	arch_flash_unlock();
	for (unsigned i = 0; i < 64; i++) flash_func_write_word(0x100 + 4 * i, 0x01000000u * i + i);   /* second page */
	assert(ncalls == 0);                                           /* still buffered */
	assert(flash_func_read_word(0x100 + 4 * 63) == 0x3f00003fu);  /* reading the last word flushes */
	assert(strcmp(calls, "CXPFR") == 0 && last_prog_addr == 0x20100 && last_prog_count == 256 && restore_ok);
	assert(*(volatile uint32_t *)(uintptr_t)(APP_LOAD_ADDRESS + 0x104) == 0x01000001u);
	reset_log();
	for (unsigned i = 1; i < 64; i++) flash_func_write_word(4 * i, i);   /* first page, first word last */
	assert(ncalls == 0);
	flash_func_write_word(0, 0x2007fff8u);
	assert(strcmp(calls, "CXPFR") == 0 && last_prog_addr == 0x20000 && last_prog_count == 256);
	assert(flash_func_read_word(0) == 0x2007fff8u && flash_func_read_word(4) == 1);
	assert(flash_func_read_word(2) == 0);                           /* unaligned reads as zero */
	reset_log();

	/* raw page interface guards: alignment, the bootloader's own sectors, the end of flash */
	uint8_t page[256]; memset(page, 0, sizeof(page));
	assert(arch_flash_write(APP_LOAD_ADDRESS + 4, page, 256) == 0);
	assert(arch_flash_write(APP_LOAD_ADDRESS, page, 100) == 0);
	assert(arch_flash_write(APP_LOAD_ADDRESS - 256, page, 256) == 0);
	assert(arch_flash_write(BL_FLASH_BASE + BOARD_FLASH_SIZE - 128, page, 256) == 0);
	assert(ncalls == 0);
	assert(arch_flash_write(BL_FLASH_BASE + BOARD_FLASH_SIZE - 256, page, 256) == 256 && XIP[BOARD_FLASH_SIZE - 1] == 0);
	reset_log();

	/* no usable ROM: nothing is touched */
	rom_present = false;
	XIP[0x30000] = 0;
	assert(arch_flash_write(APP_LOAD_ADDRESS, page, 256) == 256);  /* table already resolved, still works */
	reset_log();
	puts("flash ok");
}

static void test_identity(void)
{
	putreg32(0x20004927u, BL_SYSINFO_CHIP_ID);             /* RP2350 A2 */
	putreg32(0x89abcdefu, BL_UNIQUE_ID_LO);
	putreg32(0x01234567u, BL_UNIQUE_ID_HI);
	uint8_t desc[MAX_DES_LENGTH]; memset(desc, 0, sizeof(desc));
	assert(get_mcu_id() == 0x20004927u);
	int n = get_mcu_desc(sizeof(desc), desc);
	assert(n == 8 && memcmp(desc, "RP2350,2", 8) == 0);
	putreg32(0x20002927u, BL_SYSINFO_CHIP_ID);             /* RP2040 part on an RP2350 build */
	get_mcu_desc(sizeof(desc), desc);
	assert(memcmp(desc, "RP2350,?", 8) == 0);
	n = get_mcu_desc(4, desc);
	assert(n == 3 && memcmp(desc, "RP2", 3) == 0);          /* respects max */
	assert(check_silicon() == 0);
	assert(flash_func_read_sn(0) == 0x89abcdefu && flash_func_read_sn(4) == 0x01234567u && flash_func_read_sn(8) == 0);
	assert(flash_func_read_otp(0) == 0);
	assert(board_info.board_type == BOARD_TYPE && board_info.systick_mhz == 150);
	puts("identity ok");
}

static void test_systick_leds(void)
{
	putreg32(0, NVIC_SYSTICK_CTRL);
	board_info.systick_mhz = 150;
	arch_systic_init();
	assert(getreg32(NVIC_SYSTICK_RELOAD) == 149999);
	assert((getreg32(NVIC_SYSTICK_CTRL) & 7) == (NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT | NVIC_SYSTICK_CTRL_ENABLE));
	arch_systic_deinit();
	assert((getreg32(NVIC_SYSTICK_CTRL) & 3) == 0 && getreg32(NVIC_SYSTICK_RELOAD) == 0);
	systick_set_reload(0x2000000);                          /* 24-bit field */
	assert(getreg32(NVIC_SYSTICK_RELOAD) == 0xffffff);
	putreg32(NVIC_SYSTICK_CTRL_COUNTFLAG, NVIC_SYSTICK_CTRL);
	assert(systick_get_countflag() == 1);

	arch_setvtor((const uint32_t *)(uintptr_t)APP_LOAD_ADDRESS);
	assert(getreg32(NVIC_VECTAB) == APP_LOAD_ADDRESS);

	putreg32(NVIC_SYSTICK_CTRL_ENABLE | NVIC_SYSTICK_CTRL_TICKINT, NVIC_SYSTICK_CTRL);
	putreg32(0, NVIC_INTCTRL);
	clock_deinit();
	assert((getreg32(NVIC_SYSTICK_CTRL) & 3) == 0);
	assert(getreg32(NVIC_INTCTRL) == (NVIC_INTCTRL_PENDSTCLR | NVIC_INTCTRL_PENDSVCLR));

	/* LEDs: active low pins 0 and 1, outputs after init, inputs after deinit */
	led_on(LED_BOOTLOADER); led_off(LED_ACTIVITY);
	assert(gpio_level[1] == 0 && gpio_level[0] == 1);
	led_toggle(LED_BOOTLOADER); led_toggle(LED_ACTIVITY);
	assert(gpio_level[1] == 1 && gpio_level[0] == 0);
	putreg32(0, BL_RESETS_SET);
	board_deinit();
	assert(gpio_sio[0] && gpio_sio[1] && !gpio_out[0] && !gpio_out[1]);
	assert(getreg32(BL_RESETS_SET) == BL_RESETS_USBCTRL);
	assert(board_get_devices() == USB0_DEV);
	puts("systick ok");
}

int main(int argc, char **argv)
{
	map(BL_FLASH_BASE, BOARD_FLASH_SIZE);
	map(RP23XX_SYSINFO_BASE, 4096);
	map(BL_RESETS_SET, 4096);
	map(RP23XX_WATCHDOG_BASE, 4096);
	map(BL_BOOTRAM_BASE, 4096);
	map(RP23XX_OTP_DATA_BASE, 4096);
	map(0xe000e000u, 4096);

	for (unsigned i = 0; i < 64; i++) xip_image[i] = 0xb5000000u + i;
	memcpy((void *)(uintptr_t)BL_BOOTRAM_BASE, xip_image, sizeof(xip_image));
	xip_setup_ptr_holder = BL_BOOTRAM_BASE;

	const char *scenario = argc > 1 ? argv[1] : "";
	if (strcmp(scenario, "units") == 0) {
		test_flash();
		test_identity();
		test_systick_leds();
		return 0;
	}
	if (strcmp(scenario, "boot") == 0) {
		putreg32(0, BL_BOOT_SIGNATURE_REG);
	} else if (strcmp(scenario, "boot_signature") == 0) {
		putreg32(0xb007b007u, BL_BOOT_SIGNATURE_REG);
	} else if (strcmp(scenario, "boot_vbus") == 0) {
		putreg32(0, BL_BOOT_SIGNATURE_REG);
		gpio_level[24] = 1;
	} else {
		fprintf(stderr, "unknown scenario %s\n", scenario);
		return 2;
	}
	bootloader_main(0, NULL);
	return 1;   /* bootloader() exits */
}
'''

SOURCES = (RPI_COMMON / "flash.c", RPI_COMMON / "main.c", RPI_COMMON / "systick.c",
           BOOTLOADER / "common/lib/flash_cache.c")


def build(temp, extra_flags=()):
    temp = pathlib.Path(temp)
    for name, text in STUBS.items():
        path = temp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    includes = [temp, BOOTLOADER / "common", RPI_COMMON.parent / "rp2350/include",
                NUTTX_ARCH / "rp23xx", NUTTX_ARCH / "armv8-m"]
    # The firmware build force-includes nuttx/config.h, which flash_cache.h needs to pick the
    # page size; NuttX's <string.h> also brings in the fixed-width types bl.h relies on.
    flags = [f"-I{p}" for p in includes] + ["-include", "nuttx/config.h", "-include", "stdint.h", "-include", "stdbool.h",
                                             "-include", "stddef.h", "-include", "sys/types.h", "-std=gnu11", "-Wall", "-Wextra",
                                             "-Wno-unused-parameter", "-Werror", "-g", "-O2", *extra_flags]  # -O2: the common cache uses C99 inline
    objects = []
    for source in SOURCES:
        obj = temp / (source.stem + ".o")
        subprocess.run(["cc", *flags, "-c", str(source), "-o", str(obj)], check=True)
        objects.append(str(obj))
    test = temp / "test.c"
    test.write_text(TEST_C)
    exe = temp / "test"
    subprocess.run(["cc", *flags, "-Wno-error=unused-function", str(test), *objects, "-o", str(exe)], check=True)
    return exe


def run(exe, scenario):
    result = subprocess.run([str(exe), scenario], capture_output=True, text=True)
    if result.returncode != 0:
        raise AssertionError(f"{scenario}: exit {result.returncode}\n{result.stdout}{result.stderr}")
    return result.stdout


class BootloaderTest(unittest.TestCase):
    def test_units(self):
        with tempfile.TemporaryDirectory(prefix="rpi-bl-test-") as temp:
            out = run(build(temp), "units")
            self.assertEqual(out.split(), ["flash", "ok", "identity", "ok", "systick", "ok"])

    def test_boot_decision(self):
        with tempfile.TemporaryDirectory(prefix="rpi-bl-test-") as temp:
            exe = build(temp)
            # No VBUS sense: always wait BOOTLOADER_DELAY on USB before booting
            self.assertEqual(run(exe, "boot").strip(), "bootloader timeout=5000 scratch=0 jumped=0 cinit=1")
            # Application asked for the bootloader: no timeout, request cleared
            self.assertEqual(run(exe, "boot_signature").strip(), "bootloader timeout=0 scratch=0 jumped=0 cinit=1")

    def test_boot_decision_with_vbus(self):
        with tempfile.TemporaryDirectory(prefix="rpi-bl-test-") as temp:
            exe = build(temp, ["-DTEST_VBUS"])
            # VBUS absent: try the application first; it is not bootable here, so stay forever
            self.assertEqual(run(exe, "boot").strip(), "bootloader timeout=0 scratch=0xb007b007 jumped=1 cinit=1")
            # VBUS present: wait for an upload with the usual timeout
            self.assertEqual(run(exe, "boot_vbus").strip(), "bootloader timeout=5000 scratch=0 jumped=0 cinit=1")
            self.assertEqual(run(exe, "boot_signature").strip(), "bootloader timeout=0 scratch=0 jumped=0 cinit=1")


if __name__ == "__main__":
    unittest.main()
