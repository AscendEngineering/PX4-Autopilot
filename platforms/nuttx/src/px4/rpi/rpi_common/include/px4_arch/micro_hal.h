#pragma once

#include <px4_platform/micro_hal.h>

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/irq.h>

__BEGIN_DECLS

// The chip-specific rpi_/RPI_ names used below are defined by the per-chip
// px4_arch/micro_hal.h (rp2040/ or rp2350/) that includes this file.

// RP2040 and RP2350 don't have a bbsram. Following two defines are copied from nxp/k66.
// This will remove the errors of undefined PX4_BBSRAM_SIZE when logger module is activated.
// Fixme: using ??
#define PX4_BBSRAM_SIZE             2048
#define PX4_HF_GETDESC_IOCTL        0

// RP2350 provides a 64-bit unique id in OTP; RP2040 has none and uses a fixed value.
// Take a look at board_identity.c file in version folder.

#define PX4_CPU_UUID_BYTE_LENGTH                12
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It will be converted to a string with the MSD on left and LSD on the right most position.
 */
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

/* By not defining PX4_CPU_UUID_CORRECT_CORRELATION the following maintains the legacy incorrect order
 * used for selection of significant digits of the UUID in the PX4 code base.
 * This is done to avoid the ripple effects changing the IDs used on STM32 base platforms
 */
#if defined(PX4_CPU_UUID_CORRECT_CORRELATION)
# define PX4_CPU_UUID_WORD32_UNIQUE_H            0 /* Least significant digits change the most */
# define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
# define PX4_CPU_UUID_WORD32_UNIQUE_L            2 /* Most significant digits change the least */
#else
/* Legacy incorrect ordering */
# define PX4_CPU_UUID_WORD32_UNIQUE_H            2 /* Most significant digits change the least */
# define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
# define PX4_CPU_UUID_WORD32_UNIQUE_L            0 /* Least significant digits change the most */
#endif

/*                                                  Separator    nnn:nnn:nnnn     2 char per byte           term */
#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

#define PX4_BUS_OFFSET       1                  /* RP2040/RP2350 buses are 0 based and adjustment is needed */
#define px4_spibus_initialize(bus_num_1based)   rpi_spibus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))

#define px4_i2cbus_initialize(bus_num_1based)   rpi_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
#define px4_i2cbus_uninitialize(pdev)           rpi_i2cbus_uninitialize(pdev)

// RP2040 and RP2350 do not have the gpio configuration process similar to stm or tiva devices.
// There are multiple different registers which are required to be configured based on the function selection.
// However, only five values are required for the most part: Pin number, Pull up/down, direction, set/clear and function
//
// The pinset below is defined using a 16-bit value where,
// bits		Function
// 0-5		GPIO number. 0-29 is valid on RP2040 and RP2350A, 0-47 on RP2350B.
// 6		Pull up
// 7		Pull down
// 8		Direction
// 9		Set/clear
// 10-14	GPIO function select
// 15		Unused
//
// The pin field is six bits wide so that one layout covers the 48 GPIOs of
// RP2350B; rpi_gpioconfig() rejects pins the running chip does not have.
#define GPIO_PU		(1 << 6)	// Pull-up resistor
#define GPIO_PD		(1 << 7)	// Pull-down resistor
#define GPIO_OUT	(1 << 8)	// Output enable
#define GPIO_SET	(1 << 9)	// Output set
#define GPIO_FUN(func)	(func << 10)	// Function select

#define GPIO_NUM_MASK	0x3f		// GPIO pin number mask
#define	GPIO_PU_MASK	0x40		// Pull-up enable mask
#define	GPIO_PD_MASK	0x80		// Pull-down enable mask
#define	GPIO_OUT_MASK	0x100		// Output enable mask
#define	GPIO_SET_MASK	0x200		// Output set mask
#define	GPIO_FUN_MASK	0x7c00		// GPIO function select mask
#define GPIO_FUN_SHIFT	10		// GPIO function select position

int rpi_gpioconfig(uint32_t pinset);
int rpi_setgpioevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg);

#define px4_arch_configgpio(pinset)		rpi_gpioconfig(pinset)				// Defined in io_pins/rpi_pinset.c
#define px4_arch_unconfiggpio(pinset)           rpi_gpio_init(pinset & GPIO_NUM_MASK)		// Reset the pin as input SIO
#define px4_arch_gpioread(pinset)               rpi_gpio_get(pinset & GPIO_NUM_MASK)		// Use gpio_get
#define px4_arch_gpiowrite(pinset, value)       rpi_gpio_put(pinset & GPIO_NUM_MASK, value)	// Use gpio_put
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a) rpi_setgpioevent(pinset,r,f,e,fp,a)		// Defined in io_pins/rpi_pinset.c

// Following are quick defines to be used with the functions defined above
// These defines create a bit-mask which is supposed to be used in the
// functions defined above to set up gpios correctly.
#define PX4_MAKE_GPIO_INPUT(gpio) (gpio | GPIO_PU | GPIO_FUN(RPI_GPIO_FUNC_SIO))
#define PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio) (gpio | GPIO_OUT | GPIO_FUN(RPI_GPIO_FUNC_SIO))
#define PX4_MAKE_GPIO_OUTPUT_SET(gpio) (gpio | GPIO_OUT | GPIO_SET | GPIO_FUN(RPI_GPIO_FUNC_SIO))

#define PX4_GPIO_PIN_OFF(pinset) ((pinset & GPIO_NUM_MASK) | GPIO_FUN(RPI_GPIO_FUNC_SIO) | GPIO_PD)

#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc malloc

__END_DECLS
