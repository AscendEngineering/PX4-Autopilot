/*
 * RP2350 bare-metal probe: shared definitions.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

/* Build role, set by the Makefile. */
#ifndef PROBE_ROLE_APP
#define PROBE_ROLE_APP 0
#endif

/* Processor clock assumed for SysTick (ROM leaves no PLL running). */
#ifndef PROBE_CPU_HZ
#define PROBE_CPU_HZ 12000000u
#endif

/* Flash map (ref 3.1). */
#define XIP_BASE            0x10000000u
#define BL_FLASH_BASE       0x10000000u
#define APP_FLASH_BASE      0x10020000u
#define APP_FLASH_END       0x103f0000u   /* params region starts here */
#define FLASH_SECTOR_SIZE   4096u
#define FLASH_PAGE_SIZE     256u
#define PROBE_TEST_SECTOR   1007u         /* 0x103ef000, top of the app region */

/* SRAM (ds 2.2.3). */
#define SRAM_BASE           0x20000000u
#define SRAM_END            0x20080000u   /* end of striped SRAM0-7 */
#define SCRATCH_X_BASE      0x20080000u
#define SCRATCH_Y_BASE      0x20081000u
#define SCRATCH_Y_END       0x20082000u
#define APP_MSP_MAX         0x2007fff8u

/* Peripheral bases (ds 2.2). */
#define SYSINFO_BASE        0x40000000u
#define IO_BANK0_BASE       0x40028000u
#define PADS_BANK0_BASE     0x40038000u
#define WATCHDOG_BASE       0x400d8000u
#define BOOTRAM_BASE        0x400e0000u
#define SIO_BASE            0xd0000000u

#define SYSINFO_CHIP_ID     (*(volatile uint32_t *)(SYSINFO_BASE + 0x00))
#define WATCHDOG_SCRATCH0   (*(volatile uint32_t *)(WATCHDOG_BASE + 0x0c))

/* GPIO: IO_BANK0 CTRL, PADS_BANK0, SIO. */
#define IO_BANK0_GPIO_CTRL(n)   (*(volatile uint32_t *)(IO_BANK0_BASE + 0x04 + (n) * 8))
#define IO_BANK0_FUNCSEL_SIO    5u
#define PADS_BANK0_GPIO(n)      (*(volatile uint32_t *)(PADS_BANK0_BASE + 0x04 + (n) * 4))
#define PADS_ISO                (1u << 8)
#define PADS_OD                 (1u << 7)
#define PADS_IE                 (1u << 6)
#define PADS_DRIVE_4MA          (1u << 4)
#define PADS_PUE                (1u << 3)
#define PADS_PDE                (1u << 2)
#define SIO_GPIO_OUT_SET        (*(volatile uint32_t *)(SIO_BASE + 0x18))
#define SIO_GPIO_OUT_CLR        (*(volatile uint32_t *)(SIO_BASE + 0x20))
#define SIO_GPIO_OUT_XOR        (*(volatile uint32_t *)(SIO_BASE + 0x28))
#define SIO_GPIO_OE_SET         (*(volatile uint32_t *)(SIO_BASE + 0x38))

/* TIMER1 counts clk_sys cycles independently of interrupts (ds 12.8.5).
 * TIMER0 is left available to the ROM. Reset aliases are from ds 2.1.3/7.5.
 */
#define RESETS_BASE             0x40020000u
#define RESETS_SET              (*(volatile uint32_t *)(RESETS_BASE + 0x2000))
#define RESETS_CLR              (*(volatile uint32_t *)(RESETS_BASE + 0x3000))
#define RESETS_DONE             (*(volatile uint32_t *)(RESETS_BASE + 0x08))
#define RESETS_TIMER1           (1u << 24)
#define RESETS_SYSINFO          (1u << 21)
#define RESETS_PADS_BANK0       (1u << 9)
#define RESETS_IO_BANK0         (1u << 6)
#define TIMER1_BASE             0x400b8000u
#define TIMER1_RAW_LOW          (*(volatile uint32_t *)(TIMER1_BASE + 0x28))
#define TIMER1_DBGPAUSE         (*(volatile uint32_t *)(TIMER1_BASE + 0x2c))
#define TIMER1_SOURCE           (*(volatile uint32_t *)(TIMER1_BASE + 0x38))
#define TIMER_SOURCE_CLK_SYS    1u

/* Board LEDs. Default: RPI-UAVFC-R4 (schematic U1 pins 77/78), active low.
 * BOARD=pico2 in the Makefile overrides to the Pico 2 LED on GPIO25, active high,
 * with "green" on an unused header pin. */
#ifndef LED_BLUE_GPIO
#define LED_BLUE_GPIO       0u
#endif
#ifndef LED_GREEN_GPIO
#define LED_GREEN_GPIO      1u
#endif
#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH     0
#endif
#if LED_ACTIVE_HIGH
#define LED_ON(mask)        (SIO_GPIO_OUT_SET = (mask))
#define LED_OFF(mask)       (SIO_GPIO_OUT_CLR = (mask))
#else
#define LED_ON(mask)        (SIO_GPIO_OUT_CLR = (mask))
#define LED_OFF(mask)       (SIO_GPIO_OUT_SET = (mask))
#endif

/* Cortex-M33 system control (ARMv8-M). */
#define SYST_CSR            (*(volatile uint32_t *)0xe000e010u)
#define SYST_RVR            (*(volatile uint32_t *)0xe000e014u)
#define SYST_CVR            (*(volatile uint32_t *)0xe000e018u)
#define SYST_CSR_ENABLE     (1u << 0)
#define SYST_CSR_TICKINT    (1u << 1)
#define SYST_CSR_CLKSOURCE  (1u << 2)
#define NVIC_ICER(n)        (*(volatile uint32_t *)(0xe000e180u + (n) * 4))
#define NVIC_ICPR(n)        (*(volatile uint32_t *)(0xe000e280u + (n) * 4))
#define SCB_ICSR            (*(volatile uint32_t *)0xe000ed04u)
#define SCB_VTOR            (*(volatile uint32_t *)0xe000ed08u)
#define SCB_ICSR_PENDSTCLR  (1u << 25)
#define SCB_ICSR_PENDSVCLR  (1u << 27)

/* Status block (spec 1.4), at 0x20081000. */
#define STATUS_MAGIC_BL     0x50524f42u   /* "PROB" */
#define STATUS_MAGIC_APP    0x41505050u   /* "APPP" */

struct status_block {
	uint32_t magic;
	uint32_t step;              /* 0x1a, 0x1b, 0x1c */
	uint32_t chip_id;
	uint32_t image_def[5];
	uint32_t app_first_word;
	uint32_t scratch0;
	uint32_t rom_fn[6];         /* connect, exit_xip, erase, program, flush, get_sys_info */
	uint32_t device_id[2];
	uint32_t sram_test;         /* bit per address, 1 = pass */
	uint32_t flash_test;        /* 0 not run, 1 pass, 2 erase readback, 3 program readback */
	uint32_t erase_cycles;
	uint32_t program_cycles;
	uint32_t jump_state;        /* 0 not attempted, 1 rejected, 2 accepted */
	uint32_t vtor;
	uint32_t systick_irqs;
	/* Additions beyond spec 1.4, appended so earlier offsets hold. */
	uint32_t xip_setup_ptr;     /* resolved XIP setup function image address */
	uint32_t fault;             /* 0, or 0xdead0000 | exception number */
};

extern volatile struct status_block status;

/* start.c */
extern uint32_t _stack_bottom, _stack_top;
extern const uint32_t vector_table[];
extern volatile uint32_t sram_test_lo[2];   /* 0x20000000 */
extern volatile uint32_t sram_test_hi[2];   /* 0x2007fff8 */
extern volatile uint32_t scratch_x_test[2]; /* 0x20080000 */
extern volatile uint32_t scratch_y_test[2]; /* 0x20081ff8 */

static inline void dsb(void) { __asm volatile("dsb sy" ::: "memory"); }
static inline void isb(void) { __asm volatile("isb sy" ::: "memory"); }
static inline void irq_disable(void) { __asm volatile("cpsid i" ::: "memory"); }
static inline void irq_enable(void) { __asm volatile("cpsie i" ::: "memory"); }
static inline uint32_t primask_save_disable(void)
{
	uint32_t p;
	__asm volatile("mrs %0, primask\n\tcpsid i" : "=r"(p) :: "memory");
	return p;
}
static inline void primask_restore(uint32_t p)
{
	__asm volatile("msr primask, %0" :: "r"(p) : "memory");
}
