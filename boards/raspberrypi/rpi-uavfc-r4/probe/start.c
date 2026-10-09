/*
 * RP2350 bare-metal probe: vector table, IMAGE_DEF block and reset handler.
 * Thrown away in phase 2; NuttX supplies all of this.
 */
#include "probe.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);
void SysTick_Handler(void);

void Reset_Handler(void);
static void Default_Handler(void);
static void HardFault_Handler(void);

/* Probe-only SRAM boundary test storage (spec 1.2). */
volatile uint32_t sram_test_lo[2]   __attribute__((section(".sram_lo_test")));
volatile uint32_t sram_test_hi[2]   __attribute__((section(".sram_hi_test")));
volatile uint32_t scratch_x_test[2] __attribute__((section(".scratch_x_test")));
volatile uint32_t scratch_y_test[2] __attribute__((section(".scratch_y_test")));

/* 16 core exceptions + 52 RP2350 external interrupts (ds Table 95). */
#define NUM_IRQ 52

__attribute__((section(".vectors"), used))
const uint32_t vector_table[16 + NUM_IRQ] = {
	(uint32_t) &_stack_top,           /* 0  initial MSP */
	(uint32_t)Reset_Handler,          /* 1  reset */
	(uint32_t)Default_Handler,        /* 2  NMI */
	(uint32_t)HardFault_Handler,      /* 3  HardFault */
	(uint32_t)HardFault_Handler,      /* 4  MemManage */
	(uint32_t)HardFault_Handler,      /* 5  BusFault */
	(uint32_t)HardFault_Handler,      /* 6  UsageFault */
	(uint32_t)HardFault_Handler,      /* 7  SecureFault */
	0, 0, 0,                          /* 8-10 reserved */
	(uint32_t)Default_Handler,        /* 11 SVCall */
	(uint32_t)Default_Handler,        /* 12 DebugMonitor */
	0,                                /* 13 reserved */
	(uint32_t)Default_Handler,        /* 14 PendSV */
	(uint32_t)SysTick_Handler,        /* 15 SysTick */
	/* external interrupts: all default */
	[16 ...(16 + NUM_IRQ - 1)] = (uint32_t)Default_Handler,
};

/*
 * Minimum "flash image boot" block loop (ds 5.9.5.1), immediately after the
 * vector table so it sits well inside the first 4 KB.
 */
__attribute__((section(".image_def"), used))
const uint32_t image_def[5] = {
	0xffffded3u,   /* PICOBIN_BLOCK_MARKER_START */
	0x10210142u,   /* IMAGE_TYPE: EXE, Arm, Secure, RP2350 */
	0x000001ffu,   /* LAST item, size 1 */
	0x00000000u,   /* link to self */
	0xab123579u,   /* PICOBIN_BLOCK_MARKER_END */
};

void Reset_Handler(void)
{
	/* Mirror the post-jump state: interrupts masked until main() has set up SysTick. */
	irq_disable();

	/* Own the vector table before anything can fault. */
	SCB_VTOR = (uint32_t)vector_table;
	dsb();
	isb();

	/* Copy .data (including .ramfunc) from flash, zero .bss. Plain loops: no libc. */
	volatile uint32_t *dst = &_sdata;
	const volatile uint32_t *src = &_sidata;

	while (dst < &_edata) {
		*dst++ = *src++;
	}

	dst = &_sbss;

	while (dst < &_ebss) {
		*dst++ = 0;
	}

	main();

	for (;;) {
		__asm volatile("wfi");
	}
}

static void Default_Handler(void)
{
	uint32_t ipsr;
	__asm volatile("mrs %0, ipsr" : "=r"(ipsr));
	status.fault = 0xdead0000u | (ipsr & 0x1ffu);

	for (;;) {
		__asm volatile("wfi");
	}
}

static void HardFault_Handler(void)
{
	uint32_t ipsr;
	__asm volatile("mrs %0, ipsr" : "=r"(ipsr));
	status.fault = 0xdead0000u | (ipsr & 0x1ffu);

	/* Both LEDs on solid: visible without SWD. */
	LED_ON((1u << LED_BLUE_GPIO) | (1u << LED_GREEN_GPIO));

	for (;;) {
		__asm volatile("wfi");
	}
}
