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
 * @file main.c
 *
 * PX4 bootloader board support for RP2040/RP2350.
 *
 * Provides the chip side of the common bootloader interface (bl.h): board
 * identity, LEDs, the boot-to-bootloader signature, SysTick start and stop,
 * the hand-off to the application and bootloader_main() itself. Flash lives
 * in flash.c. Chip specifics come from bl_chip.h in the chip directory.
 *
 * The board's hw_config.h supplies, in addition to the common names used by
 * every PX4 bootloader board (BOARD_TYPE, APP_LOAD_ADDRESS, BOOTLOADER_DELAY,
 * BOARD_FLASH_SIZE, BOARD_FLASH_SECTORS, BOARD_FIRST_FLASH_SECTOR_TO_ERASE,
 * APP_RESERVATION_SIZE, INTERFACE_USB, INTERFACE_USB_CONFIG,
 * BOOT_DEVICES_SELECTION, BOOT_DEVICES_FILTER_ONUSB):
 *
 *   BOARD_PIN_LED_ACTIVITY, BOARD_PIN_LED_BOOTLOADER  GPIO numbers (optional)
 *   BOARD_LED_ON, BOARD_LED_OFF                       pin level for each state
 *   BOARD_VBUS                                        GPIO number (optional)
 *
 * There is no force-bootloader pin: BOOTSEL sits on the QSPI chip select,
 * which cannot be sampled while XIP is live, and the ROM already honours it
 * before this code runs.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <nuttx/config.h>
#include <arch/board/board.h>
#include <arm_internal.h>
#include <nvic.h>

#include "hw_config.h"
#include "bl.h"
#include "bl_chip.h"
#include <lib/systick.h>
#include <lib/flash_cache.h>

#if !defined(BOOTLOADER_RESERVATION_SIZE)
#  define BOOTLOADER_RESERVATION_SIZE	(128 * 1024)
#endif

#if defined(FLASH_BASED_PARAMS) && (APP_RESERVATION_SIZE <= 0)
#  error "APP_RESERVATION_SIZE must be greater than 0 if FLASH_BASED_PARAMS is defined"
#endif

#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

#if INTERFACE_USART
#  error "The rpi bootloader has no USART interface"
#endif

#define BOOT_RTC_SIGNATURE		0xb007b007

/* SYSINFO CHIP_ID, same layout on both chips (JEP-106):
 *   REVISION [31:28]  PART [27:12]  MANUFACTURER [11:1]  STOP_BIT [0] */
#define CHIP_ID_MANUFACTURER_MASK	0x00000fff
#define CHIP_ID_MANUFACTURER_RPI	0x927
#define CHIP_ID_PART_SHIFT		12
#define CHIP_ID_PART_MASK		0xffff
#define CHIP_ID_REVISION_SHIFT		28
#define CHIP_ID_REVISION_MASK		0xf

struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,
	.systick_mhz	= BOARD_SYS_FREQ / 1000000,
};

static bool usb_connected = false;

/* Boot signature in a watchdog scratch register (bl_chip.h). Read once by
 * bootloader_main() and cleared, so a crash or watchdog reset never leaves
 * the board parked in the bootloader. */

static uint32_t board_get_boot_signature(void)
{
	return getreg32(BL_BOOT_SIGNATURE_REG);
}

static void board_set_boot_signature(uint32_t sig)
{
	putreg32(sig, BL_BOOT_SIGNATURE_REG);
}

uint32_t board_get_devices(void)
{
	uint32_t devices = BOOT_DEVICES_SELECTION;

	if (usb_connected) {
		devices &= BOOT_DEVICES_FILTER_ONUSB;
	}

	return devices;
}

static void board_init(void)
{
	board_info.fw_size = APP_SIZE_MAX;

#if defined(BOARD_VBUS)
	bl_gpio_input(BOARD_VBUS);
#endif

#if defined(BOARD_PIN_LED_ACTIVITY)
	bl_gpio_output(BOARD_PIN_LED_ACTIVITY, BOARD_LED_OFF);
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
	bl_gpio_output(BOARD_PIN_LED_BOOTLOADER, BOARD_LED_OFF);
#endif
}

/* Leave the hardware as the application's NuttX start-up expects to find it.
 * Holding the USB controller in reset drops the host connection, so the
 * application's CDC ACM enumerates afresh. */
void board_deinit(void)
{
#if defined(BOARD_PIN_LED_ACTIVITY)
	bl_gpio_input(BOARD_PIN_LED_ACTIVITY);
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
	bl_gpio_input(BOARD_PIN_LED_BOOTLOADER);
#endif
#if defined(BOARD_VBUS)
	bl_gpio_input(BOARD_VBUS);
#endif

#if INTERFACE_USB
	putreg32(BL_RESETS_USBCTRL, BL_RESETS_SET);
#endif
}

void arch_systic_init(void)
{
	systick_set_clocksource(CLKSOURCE_PROCESOR);
	systick_set_reload(board_info.systick_mhz * 1000);	/* 1 ms */
	systick_interrupt_enable();
	systick_counter_enable();
}

void arch_systic_deinit(void)
{
	systick_interrupt_disable();
	systick_counter_disable();
	systick_set_reload(0);
}

/* Clocks stay as NuttX configured them: the application's rp23xx_clockconfig
 * resets the PLLs and clock muxes from scratch. Only the SysTick state the
 * common code did not already clear is tidied here, with interrupts masked
 * from this point until the application runs. */
void clock_deinit(void)
{
	(void)bl_irq_save();
	modifyreg32(NVIC_SYSTICK_CTRL, NVIC_SYSTICK_CTRL_ENABLE | NVIC_SYSTICK_CTRL_TICKINT, 0);
	putreg32(0, NVIC_SYSTICK_CURRENT);
	putreg32(NVIC_INTCTRL_PENDSTCLR | NVIC_INTCTRL_PENDSVCLR, NVIC_INTCTRL);
}

void arch_setvtor(const uint32_t *address)
{
	putreg32((uint32_t)(uintptr_t)address, NVIC_VECTAB);
	bl_barrier();
}

uint32_t get_mcu_id(void)
{
	return getreg32(BL_SYSINFO_CHIP_ID);
}

/* "RP2350,2" style: chip name, then the raw silicon revision digit from
 * CHIP_ID, the same value the application reports through board_mcu_version. */
int get_mcu_desc(int max, uint8_t *revstr)
{
	const uint32_t chip_id = get_mcu_id();
	const char *name = BL_CHIP_NAME;
	char rev = '?';

	if ((chip_id & CHIP_ID_MANUFACTURER_MASK) == CHIP_ID_MANUFACTURER_RPI &&
	    ((chip_id >> CHIP_ID_PART_SHIFT) & CHIP_ID_PART_MASK) == BL_CHIP_ID_PART) {
		const unsigned revision = (chip_id >> CHIP_ID_REVISION_SHIFT) & CHIP_ID_REVISION_MASK;
		rev = revision < 10 ? '0' + revision : '?';
	}

	uint8_t *endp = &revstr[max - 1];
	uint8_t *strp = revstr;

	while (strp < endp && *name) {
		*strp++ = *name++;
	}

	if (strp < endp) {
		*strp++ = ',';
	}

	if (strp < endp) {
		*strp++ = rev;
	}

	return strp - revstr;
}

int check_silicon(void)
{
	return 0;
}

/* Serial number words 0 and 1 are the 64-bit OTP device id, in the order the
 * application uses for its UUID; anything past that reads as zero. */
uint32_t flash_func_read_sn(uintptr_t address)
{
	switch (address) {
	case 0:
		return getreg32(BL_UNIQUE_ID_LO);

	case 4:
		return getreg32(BL_UNIQUE_ID_HI);

	default:
		return 0;
	}
}

uint32_t flash_func_read_otp(uintptr_t address)
{
	(void)address;
	return 0;
}

static void led_set_level(unsigned led, int level)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		bl_gpio_put(BOARD_PIN_LED_ACTIVITY, level);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		bl_gpio_put(BOARD_PIN_LED_BOOTLOADER, level);
#endif
		break;
	}

	(void)level;
}

void led_on(unsigned led)
{
	led_set_level(led, BOARD_LED_ON);
}

void led_off(unsigned led)
{
	led_set_level(led, BOARD_LED_OFF);
}

void led_toggle(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		bl_gpio_put(BOARD_PIN_LED_ACTIVITY, !bl_gpio_get(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		bl_gpio_put(BOARD_PIN_LED_BOOTLOADER, !bl_gpio_get(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

/* Hand over to the application. The common code has already validated the
 * vector table, stopped SysTick and set VTOR; interrupts are masked since
 * clock_deinit(). Disable and clear every external interrupt, then finish in
 * assembly with no further C calls or stack use: privileged Thread mode on
 * the MSP, no stack limit, the application's MSP, its reset vector. The
 * application clears PRIMASK once its own start-up is ready. */
void arch_do_jump(const uint32_t *app_base)
{
	const uint32_t stacktop = app_base[APP_VECTOR_OFFSET_WORDS];
	const uint32_t entrypoint = app_base[APP_VECTOR_OFFSET_WORDS + 1];

	(void)bl_irq_save();

	for (int i = 0; i < BL_NVIC_IRQ_REGS; i++) {
		putreg32(0xffffffff, NVIC_IRQ_CLEAR(32 * i));
		putreg32(0xffffffff, NVIC_IRQ_CLRPEND(32 * i));
	}

	bl_barrier();

#if defined(__ARM_ARCH)
	uint32_t scratch;
	__asm__ volatile(
		"mov %0, #0          \n"
		"msr basepri, %0     \n"
		"msr faultmask, %0   \n"
		"msr control, %0     \n"	/* privileged, MSP, no FP context */
		"isb sy              \n"
		"msr msplim, %0      \n"
		"msr msp, %1         \n"
		"bx %2               \n"
		: "=&r"(scratch) : "r"(stacktop), "r"(entrypoint) : "memory");
#else
	(void)stacktop;
	(void)entrypoint;
#endif

	for (;;);
}

int bootloader_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	bool try_boot = true;			/* try booting before we drop to the bootloader */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

	board_init();

	/* The application asked for the bootloader: stay until something is
	 * uploaded, and clear the request so the next reset boots normally. */
	if (board_get_boot_signature() == BOOT_RTC_SIGNATURE) {
		try_boot = false;
		timeout = 0;
		board_set_boot_signature(0);
	}

#if INTERFACE_USB
#if defined(BOARD_VBUS)

	if (bl_gpio_get(BOARD_VBUS)) {
		usb_connected = true;
		try_boot = false;
	}

#else
	try_boot = false;
#endif
#endif

	if (try_boot) {
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_boot_signature(BOOT_RTC_SIGNATURE);
#endif
		jump_to_app();

		/* Not bootable: stay here until an upload fixes that. */
		board_set_boot_signature(BOOT_RTC_SIGNATURE);
		timeout = 0;
	}

#if INTERFACE_USB
	cinit(INTERFACE_USB_CONFIG, USB);
#endif

	while (1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader(timeout);

#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_boot_signature(BOOT_RTC_SIGNATURE);
#endif
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
}
