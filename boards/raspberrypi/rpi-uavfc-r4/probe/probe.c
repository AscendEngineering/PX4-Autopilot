/*
 * RP2350 bare-metal probe: main (spec section 1.3).
 *
 * Bootloader role (PROBE_ROLE_APP=0, linked at 0x10000000):
 *   1a boot and inspect, 1b flash through the ROM, 1c validate and jump.
 * Application role (PROBE_ROLE_APP=1, linked at 0x10020000):
 *   1a only, then blink forever.
 */
#include "probe.h"
#include "rom.h"
#include "flash.h"

volatile struct status_block status __attribute__((section(".status")));

/* SysTick: 1 ms nominal tick on the processor clock (CLKSOURCE=1). */
#define SYSTICK_RELOAD   (PROBE_CPU_HZ / 1000u - 1u)
#ifndef APP_BLINK_TICKS
#define APP_BLINK_TICKS  125u                              /* app LED: 4 Hz by default */
#endif
#define BLINK_TICKS      (PROBE_ROLE_APP ? APP_BLINK_TICKS : 500u)   /* app : bootloader 1 Hz */
#define JUMP_DELAY_TICKS 5000u

static volatile uint32_t ticks;

void SysTick_Handler(void)
{
	ticks++;
	status.systick_irqs = ticks;

	if (ticks % BLINK_TICKS == 0) {
		SIO_GPIO_OUT_XOR = 1u << LED_BLUE_GPIO;
	}
}

/* IO_BANK0, PADS_BANK0 and SYSINFO power up held in reset (ds Table 535) and the
 * ROM leaves them there. Writes to a block in reset are ignored, so deassert
 * before touching GPIO or CHIP_ID. The atomic CLR alias leaves other bits alone.
 */
static void periph_unreset(void)
{
	const uint32_t mask = RESETS_IO_BANK0 | RESETS_PADS_BANK0 | RESETS_SYSINFO;

	RESETS_CLR = mask;

	while ((RESETS_DONE & mask) != mask) {}
}

static void led_init(void)
{
	const uint32_t pins[2] = { LED_BLUE_GPIO, LED_GREEN_GPIO };

	for (int i = 0; i < 2; i++) {
		uint32_t n = pins[i];
		/* Drive the off level before enabling the output. */
		LED_OFF(1u << n);
		/* Pads reset with IE=0 and ISO=1 on RP2350 (ds 9.4): enable input, clear isolation. */
		PADS_BANK0_GPIO(n) = PADS_IE | PADS_DRIVE_4MA;
		IO_BANK0_GPIO_CTRL(n) = IO_BANK0_FUNCSEL_SIO;
		SIO_GPIO_OE_SET = 1u << n;
	}
}

static uint32_t sram_test_word(volatile uint32_t *w)
{
	uint32_t saved = *w;
	uint32_t ok = 1;

	*w = 0xa5a5a5a5u;
	dsb();

	if (*w != 0xa5a5a5a5u) {
		ok = 0;
	}

	*w = 0x5a5a5a5au;
	dsb();

	if (*w != 0x5a5a5a5au) {
		ok = 0;
	}

	*w = saved;
	return ok;
}

static void step_1a(void)
{
	/* LEDs first, blue solid on: proves main() was reached before any ROM call.
	 * SysTick takes over the blue LED at the end of this step. */
	periph_unreset();
	led_init();
	LED_ON(1u << LED_BLUE_GPIO);

	status.magic = PROBE_ROLE_APP ? STATUS_MAGIC_APP : STATUS_MAGIC_BL;
	status.step = 0x1a;
	status.chip_id = SYSINFO_CHIP_ID;

	/* IMAGE_DEF words read back through XIP from this running image. */
	const volatile uint32_t *def = (const volatile uint32_t *)&vector_table[16 + 52];

	for (int i = 0; i < 5; i++) {
		status.image_def[i] = def[i];
	}

	status.app_first_word = *(const volatile uint32_t *)APP_FLASH_BASE;
	status.scratch0 = WATCHDOG_SCRATCH0;

	status.rom_fn[0] = (uint32_t)rom_func_lookup(ROM_FUNC_CONNECT_INTERNAL_FLASH);
	status.rom_fn[1] = (uint32_t)rom_func_lookup(ROM_FUNC_FLASH_EXIT_XIP);
	status.rom_fn[2] = (uint32_t)rom_func_lookup(ROM_FUNC_FLASH_RANGE_ERASE);
	status.rom_fn[3] = (uint32_t)rom_func_lookup(ROM_FUNC_FLASH_RANGE_PROGRAM);
	status.rom_fn[4] = (uint32_t)rom_func_lookup(ROM_FUNC_FLASH_FLUSH_CACHE);
	status.rom_fn[5] = (uint32_t)rom_func_lookup(ROM_FUNC_GET_SYS_INFO);

	rom_get_sys_info_fn gsi = (rom_get_sys_info_fn)status.rom_fn[5];

	if (gsi) {
		/* out[0] = supported flags, then CHIP_INFO: package_sel, device id low, device id high (ds 5.4.8.17). */
		uint32_t info[4] = { 0, 0, 0, 0 };
		int n = gsi(info, 4, SYS_INFO_CHIP_INFO);

		if (n >= 4 && (info[0] & SYS_INFO_CHIP_INFO)) {
			status.device_id[0] = info[2];
			status.device_id[1] = info[3];
		}
	}

	status.sram_test = (sram_test_word(&sram_test_lo[0]) << 0)
			   | (sram_test_word(&sram_test_hi[1]) << 1)
			   | (sram_test_word(&scratch_x_test[0]) << 2)
			   | (sram_test_word(&scratch_y_test[1]) << 3);

	status.vtor = SCB_VTOR;

	/* SysTick on the processor clock, then unmask interrupts (spec 1.3). */
	SYST_CSR = 0;
	SYST_RVR = SYSTICK_RELOAD;
	SYST_CVR = 0;
	SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
	irq_enable();
}

#if !PROBE_ROLE_APP

static struct flash_rom rom;
static uint8_t page_pattern[FLASH_PAGE_SIZE];   /* in SRAM: the ROM programs from this buffer */

static void cycle_counter_init(void)
{
	/* Own TIMER1, with no alarms or interrupts enabled. Reset also clears PAUSE. */
	RESETS_SET = RESETS_TIMER1;
	RESETS_CLR = RESETS_TIMER1;

	while ((RESETS_DONE & RESETS_TIMER1) == 0) {}

	TIMER1_DBGPAUSE = 0;
	TIMER1_SOURCE = TIMER_SOURCE_CLK_SYS;
	dsb();
}

/* Unsigned differences handle wrap provided an operation takes < 2^32 cycles
 * (over 28 s even at 150 MHz, well above a single-sector erase).
 */
static uint32_t cycles_now(void)
{
	return TIMER1_RAW_LOW;
}


static void step_1b(void)
{
	status.step = 0x1b;
	status.flash_test = 0;

	if (flash_rom_init(&rom) != 0) {
		return;
	}

	status.xip_setup_ptr = (uint32_t)rom.xip_setup_func;
	cycle_counter_init();

	const uint32_t offset = PROBE_TEST_SECTOR * FLASH_SECTOR_SIZE;
	const volatile uint32_t *xip = (const volatile uint32_t *)(XIP_BASE + offset);

	uint32_t t0 = cycles_now();
	flash_erase(&rom, offset, FLASH_SECTOR_SIZE);
	status.erase_cycles = cycles_now() - t0;

	for (uint32_t i = 0; i < FLASH_SECTOR_SIZE / 4; i++) {
		if (xip[i] != 0xffffffffu) {
			status.flash_test = 2;
			return;
		}
	}

	for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i++) {
		page_pattern[i] = (uint8_t)(i ^ 0x5au);
	}

	t0 = cycles_now();
	flash_program(&rom, offset, page_pattern, FLASH_PAGE_SIZE);
	status.program_cycles = cycles_now() - t0;

	const volatile uint8_t *rb = (const volatile uint8_t *)(XIP_BASE + offset);

	for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i++) {
		if (rb[i] != page_pattern[i]) {
			status.flash_test = 3;
			return;
		}
	}

	status.flash_test = 1;
	LED_ON(1u << LED_GREEN_GPIO);   /* green on: pass */
}

static int app_vectors_valid(uint32_t msp, uint32_t pc)
{
	if ((msp & 7u) != 0 || msp <= (uint32_t)&_stack_bottom || msp > APP_MSP_MAX) {
		return 0;
	}

	if ((pc & 1u) == 0) {
		return 0;
	}

	uint32_t addr = pc & ~1u;
	return addr >= APP_FLASH_BASE && addr < APP_FLASH_END;
}

/* Hand-off tail: no C calls or stack use after the MSP swap. */
__attribute__((noreturn, naked))
static void jump_tail(uint32_t msp __attribute__((unused)), uint32_t pc __attribute__((unused)))
{
	__asm volatile(
		"movs  r2, #0\n\t"
		"msr   basepri, r2\n\t"
		"cpsie f\n\t"                 /* clear FAULTMASK; PRIMASK stays set */
		"msr   control, r2\n\t"       /* privileged Thread mode, MSP */
		"isb   sy\n\t"
		"msr   msplim, r2\n\t"
		"msr   msp, r0\n\t"
		"bx    r1\n\t"
		::: "memory"
	);
}

static void step_1c(void)
{
	status.step = 0x1c;
	status.jump_state = 0;

	const volatile uint32_t *app = (const volatile uint32_t *)APP_FLASH_BASE;
	uint32_t msp = app[0];
	uint32_t pc = app[1];
	status.app_first_word = msp;

	if (!app_vectors_valid(msp, pc)) {
		status.jump_state = 1;
		return;
	}

	status.jump_state = 2;

	irq_disable();

	/* Stop SysTick and drop anything pending. */
	SYST_CSR = 0;
	SYST_CVR = 0;
	SCB_ICSR = SCB_ICSR_PENDSTCLR | SCB_ICSR_PENDSVCLR;
	NVIC_ICER(0) = 0xffffffffu;
	NVIC_ICER(1) = 0xffffffffu;
	NVIC_ICPR(0) = 0xffffffffu;
	NVIC_ICPR(1) = 0xffffffffu;

	SCB_VTOR = APP_FLASH_BASE;
	dsb();
	isb();

	jump_tail(msp, pc);
}

#endif /* !PROBE_ROLE_APP */

int main(void)
{
	/* .status is NOLOAD and shared by both images. Clear all diagnostics before
	 * publishing this image's magic or enabling interrupts, without using libc.
	 */
	volatile uint8_t *status_bytes = (volatile uint8_t *)&status;

	for (size_t i = 0; i < sizeof(status); i++) {
		status_bytes[i] = 0;
	}

	step_1a();

#if !PROBE_ROLE_APP
	step_1b();

	if (status.flash_test == 1) {
		while (ticks < JUMP_DELAY_TICKS) {
			__asm volatile("wfi");
		}

		step_1c();   /* returns only if the app vectors were rejected */
	}

#endif

	for (;;) {
		__asm volatile("wfi");
	}
}
