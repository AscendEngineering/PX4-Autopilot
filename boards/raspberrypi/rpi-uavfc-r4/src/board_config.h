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

#pragma once

/**
 * @file board_config.h
 *
 * RPI-UAVFC-R4 (RP2350B) firmware configuration. Scope of this phase: boots
 * NSH over USB, keeps parameters in flash, reboots to the PX4 bootloader.
 * The flash map is the bootloader's (src/hw_config.h).
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <hardware/rp23xx_usbctrl_regs.h>

/* LEDs: schematic U1 pin 77 GPIO0 BF_BLUE_LEDn, pin 78 GPIO1 BF_GREEN_LEDn,
 * pulled to +3V3: active low, so an output set high is off. */
#define GPIO_LED_BLUE		PX4_MAKE_GPIO_OUTPUT_SET(0)
#define GPIO_LED_GREEN		PX4_MAKE_GPIO_OUTPUT_SET(1)
#define BOARD_OVERLOAD_LED	LED_BLUE

/* board_reset() calls board_on_reset() (init.c): PWM pins low before the reset */
#define BOARD_HAS_ON_RESET	1

/* No VBUS sense GPIO is wired. The USB controller reports VBUS itself
 * (SIE_STATUS bit 0). A plain volatile read keeps this header free of
 * arm_internal.h, which not every includer has. */
static inline int board_usb_vbus_present(void)
{
	return (*(volatile uint32_t *)RP23XX_USBCTRL_REGS_SIE_STATUS & RP23XX_USBCTRL_REGS_SIE_STATUS_VBUS_DETECTED) != 0;
}
#define BOARD_ADC_USB_CONNECTED	board_usb_vbus_present()

/* Flash-based parameters in the 64 KB reservation at the top of the 4 MB
 * part (hw_config.h APP_RESERVATION_SIZE): two 32 KB pages, numbered by
 * their first 4 kB sector (1008 and 1016), served by
 * platforms/nuttx/src/px4/rpi/rpi_common/flash/rpi_progmem.c. */
#define FLASH_BASED_PARAMS
#define BOARD_USE_EXTERNAL_FLASH
#define BOARD_PARAMS_FLASH_ADDRESS	0x103f0000u
#define BOARD_PARAMS_FLASH_SIZE		(64 * 1024)
#define BOARD_PARAMS_FLASH_PAGE_SIZE	(32 * 1024)

/* High-resolution timer */
#define HRT_TIMER		1
#define HRT_TIMER_CHANNEL	1

/* PWM: four outputs on GPIO18 to 21 (src/timer_config.cpp) */
#define DIRECT_PWM_OUTPUT_CHANNELS	4

#define BOARD_DMA_ALLOC_POOL_SIZE	2048
#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE	(1024 * 3)

/* I2C1 external bus is src/i2c.cpp bus 2; pins in nuttx-config/include/board.h */

__BEGIN_DECLS

#ifndef __ASSEMBLY__

extern void rpi_spiinitialize(void);
extern void rp23xx_usbinitialize(void);
extern void board_peripheral_reset(int ms);

/* rpi_progmem.c, for src/lib/parameters/flashparams/flashfs.c */
ssize_t up_progmem_ext_getpage(size_t addr);
ssize_t up_progmem_ext_eraseblock(size_t block);
ssize_t up_progmem_ext_write(size_t addr, const void *buf, size_t count);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
