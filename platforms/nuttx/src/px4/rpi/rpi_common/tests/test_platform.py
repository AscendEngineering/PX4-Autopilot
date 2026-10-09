#!/usr/bin/env python3
"""Host regression checks for rpi_common, run once per chip (RP2040, RP2350).

Run with: python3 platforms/nuttx/src/px4/rpi/rpi_common/tests/test_platform.py
Requires Linux, cc and c++; maps fake peripheral pages in the test process.
px4_arch resolves to the chip's wrapper directory, as in the firmware build; the
NuttX register headers are the real ones from the submodule and the GPIO API is
stubbed. This does not simulate PWM waveforms or replace hardware validation.
"""
import pathlib
import subprocess
import tempfile
import unittest

PLATFORM = pathlib.Path(__file__).resolve().parents[1]
ROOT = next(p for p in PLATFORM.parents if (p / "Tools").is_dir())
NUTTX_ARCH = ROOT / "platforms/nuttx/NuttX/nuttx/arch/arm/src"

CHIPS = {
    "rp2040": dict(config="CONFIG_ARCH_CHIP_RP2040", nuttx_dir="rp2040", prefix="RP2040", fn="rp2040",
                   gpio_num=30, slices=8, sys_freq=125000000, chip_id=0x20002927, rev="2",
                   extra_config=""),
    "rp2350": dict(config="CONFIG_ARCH_CHIP_RP23XX", nuttx_dir="rp23xx", prefix="RP23XX", fn="rp23xx",
                   gpio_num=48, slices=12, sys_freq=150000000, chip_id=0x10004927, rev="1",
                   extra_config="#define CONFIG_RP23XX_RP2350B 1"),
}


def stubs(chip):
    c = CHIPS[chip]
    return {
        "nuttx/config.h": f"#pragma once\n#define {c['config']} 1\n{c['extra_config']}\n",
        "arm_internal.h": "",
        "chip.h": "",
        "nuttx/arch.h": "",
        "nuttx/irq.h": "",
        "debug.h": "",
        "queue.h": "",
        f"{c['fn']}_spi.h": "",
        f"{c['fn']}_i2c.h": "",
        "px4_platform/micro_hal.h": "",
        "drivers/drv_hrt.h": "typedef uint64_t hrt_abstime;",
        "drivers/drv_pwm_output.h": """
__BEGIN_DECLS
int up_pwm_servo_init(uint32_t);
void up_pwm_servo_arm(bool, uint32_t);
void up_pwm_servo_deinit(uint32_t);
int up_pwm_servo_set_rate(unsigned);
int up_pwm_servo_set_rate_group_update(unsigned, unsigned);
int up_pwm_servo_set(unsigned, uint16_t);
uint16_t up_pwm_servo_get(unsigned);
__END_DECLS
""",
        "systemlib/px4_macros.h": "",
        "arch/board/board.h": '#include <board_config.h>',
        "board_config.h": f"""
#pragma once
#define BOARD_SYS_FREQ {c['sys_freq']}
#define BOARD_NUM_IO_TIMERS {c['slices']}
#define DIRECT_PWM_OUTPUT_CHANNELS {2 * c['slices']}
#define CONFIG_SPI 1
#define SPI_BUS_MAX_BUS_ITEMS 2
#define BOARD_NUM_SPI_CFG_HW_VERSIONS 1
""",
        f"{c['fn']}_gpio.h": f"""
#pragma once
#define {c['prefix']}_GPIO_NUM {c['gpio_num']}
#define {c['prefix']}_GPIO_FUNC_SIO 5
#define {c['prefix']}_GPIO_FUNC_PWM 4
#define {c['prefix']}_GPIO_INTR_EDGE_LOW 4
#define {c['prefix']}_GPIO_INTR_EDGE_HIGH 8
void {c['fn']}_gpio_set_pulls(uint32_t, int, int);
void {c['fn']}_gpio_setdir(uint32_t, int);
void {c['fn']}_gpio_put(uint32_t, int);
void {c['fn']}_gpio_set_function(uint32_t, uint32_t);
int {c['fn']}_gpio_irq_attach(uint32_t, int, xcpt_t, void *);
void {c['fn']}_gpio_enable_irq(uint32_t);
void {c['fn']}_gpio_disable_irq(uint32_t);
""",
        "px4_platform_common/px4_config.h": """
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/cdefs.h>
#include <nuttx/config.h>
#include <board_config.h>
#define __EXPORT
#define OK 0
#define ERROR -1
#define PX4_SOC_ARCH_ID_UNUSED 0
#define PX4_GUID_BYTE_LENGTH 18
__BEGIN_DECLS
typedef unsigned irqstate_t;
typedef int (*xcpt_t)(int, void *, void *);
static inline irqstate_t px4_enter_critical_section(void) { return 0; }
static inline void px4_leave_critical_section(irqstate_t f) { (void)f; }
static inline uint32_t getreg32(uintptr_t a) { return *(volatile uint32_t *)a; }
__END_DECLS
#include <px4_arch/micro_hal.h>
__BEGIN_DECLS
typedef uint8_t uuid_byte_t[12];
typedef uint32_t uuid_uint32_t[3];
typedef uint8_t mfguid_t[12];
typedef uint8_t px4_guid_t[18];
void board_get_uuid32(uuid_uint32_t);
int board_get_mfguid(mfguid_t);
int board_get_px4_guid(px4_guid_t);
int board_mcu_version(char *rev, const char **revstr, const char **errata);
__END_DECLS
""",
        "px4_platform_common/defines.h": "",
    }


TEST_CPP = r'''
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_pwm_output.h>
#include <px4_arch/spi_hw_description.h>
#include <px4_arch/io_timer_hw_description.h>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <sys/mman.h>
#include <cstdio>
#include <cstdlib>
#include <px4_arch/rpi_rom.h>

static void map_fixed(uintptr_t base, size_t len)
{
	void *want = (void *)(base & ~(uintptr_t)4095);
	void *got = mmap(want, len, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	if (got != want) { perror("mmap"); exit(2); }
}

/* ROM host hooks: Task 2 replaces rpi_rom_func with a fake table */
static uintptr_t xip_setup_holder;
extern "C" bool rpi_rom_present(void) { return true; }
extern "C" void *rpi_rom_func(uint32_t code) { (void)code; return nullptr; }
extern "C" void *rpi_rom_data(uint32_t code)
{
	return code == RPI_ROM_CODE('X', 'F') ? (void *)&xip_setup_holder : nullptr;
}

static void test_rom_header()
{
	static_assert(RPI_ROM_FUNC_FLASH_RANGE_ERASE == ('R' | ('E' << 8)), "ROM code packing");
	static_assert(RPI_XIP_SETUP_BYTES == 256, "XIP setup routine is 256 bytes on both chips");
	assert(RPI_FLASH_BASE == 0x10000000u);
#if defined(CONFIG_ARCH_CHIP_RP23XX)
	map_fixed(RPI_BOOTRAM_BASE, 4096);
	xip_setup_holder = RPI_BOOTRAM_BASE + 64;
	assert(rpi_rom_xip_setup() == (const uint32_t *)(RPI_BOOTRAM_BASE + 64));   /* 'X','F' points at a pointer */
	xip_setup_holder = 0;
	assert(rpi_rom_xip_setup() == (const uint32_t *)RPI_BOOTRAM_BASE);          /* documented fallback */
	assert(RPI_BOOT_SIGNATURE_REG == RP23XX_WATCHDOG_BASE + 0x0c);              /* SCRATCH0 */
#else
	assert(rpi_rom_xip_setup() == (const uint32_t *)RPI_FLASH_BASE);            /* boot2 */
	assert(RPI_BOOT_SIGNATURE_REG == RP2040_WATCHDOG_BASE + 0x0c);
#endif
	/* the handshake register is WATCHDOG SCRATCH0 on both chips: it survives SYSRESETREQ and not power-on */
	assert((RPI_BOOT_SIGNATURE_REG & 0xfff) == 0x00c);
	assert(rpi_rom_reboot_bootsel() == -1);                                     /* no 'R','B' / 'U','B' entry in the fake */
	puts("rom header ok");
}

constexpr auto absent = initSPIDevice(1, {});
constexpr auto cs0 = initSPIDevice(1, {GPIO::Pin0});
constexpr auto device = initSPIDevice(1, {GPIO::Pin2}, {GPIO::Pin0});
constexpr auto bus = initSPIBus(SPI::Bus::SPI0, {{cs0}});
constexpr auto powered = initSPIBus(SPI::Bus::SPI0, {{device}}, {GPIO::Pin0});
constexpr auto external = initSPIBusExternal(SPI::Bus::SPI1, {{initSPIConfigExternal({GPIO::Pin0})}});
static_assert(absent.cs_gpio == 0 && absent.drdy_gpio == 0, "absent device");
static_assert(cs0.cs_gpio != 0 && cs0.drdy_gpio == 0, "GPIO0 is a valid CS");
static_assert((cs0.cs_gpio & GPIO_FUN_MASK) == GPIO_FUN(RPI_GPIO_FUNC_SIO), "CS mux");
static_assert((device.drdy_gpio & GPIO_FUN_MASK) == GPIO_FUN(RPI_GPIO_FUNC_SIO), "DRDY mux");
static_assert(bus.power_enable_gpio == 0, "no implicit power pin");
static_assert((powered.power_enable_gpio & GPIO_FUN_MASK) == GPIO_FUN(RPI_GPIO_FUNC_SIO), "power mux");
static_assert(external.devices[0].cs_gpio != 0 && external.devices[1].cs_gpio == 0, "unused external CS");
static_assert(PX4_MAKE_GPIO_OUTPUT_SET(29) == (29 | GPIO_OUT | GPIO_SET | GPIO_FUN(RPI_GPIO_FUNC_SIO)), "pinset layout");
static_assert((PX4_MAKE_GPIO_OUTPUT_SET(29) & GPIO_NUM_MASK) == 29, "pin field holds 29");

#define PAIR(t, a, b) initIOTimerChannel(io_timers, {Timer::t, Timer::ChannelA}, {GPIO::a}), \
                      initIOTimerChannel(io_timers, {Timer::t, Timer::ChannelB}, {GPIO::b})
constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
    initIOTimer(Timer::Timer0), initIOTimer(Timer::Timer1), initIOTimer(Timer::Timer2),
    initIOTimer(Timer::Timer3), initIOTimer(Timer::Timer4), initIOTimer(Timer::Timer5),
    initIOTimer(Timer::Timer6), initIOTimer(Timer::Timer7),
#if RPI_PWM_NUM_SLICES > 8
    initIOTimer(Timer::Timer8), initIOTimer(Timer::Timer9), initIOTimer(Timer::Timer10), initIOTimer(Timer::Timer11),
#endif
};
constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
    PAIR(Timer0, Pin0, Pin1), PAIR(Timer1, Pin2, Pin3), PAIR(Timer2, Pin4, Pin5),
    PAIR(Timer3, Pin6, Pin7), PAIR(Timer4, Pin8, Pin9), PAIR(Timer5, Pin10, Pin11),
    PAIR(Timer6, Pin12, Pin13), PAIR(Timer7, Pin14, Pin15),
#if RPI_PWM_NUM_SLICES > 8
    PAIR(Timer8, Pin32, Pin33), PAIR(Timer9, Pin34, Pin35), PAIR(Timer10, Pin36, Pin37), PAIR(Timer11, Pin38, Pin39),
#endif
};
constexpr io_timers_channel_mapping_t io_timers_channel_mapping = {
    {{0,2}, {2,2}, {4,2}, {6,2}, {8,2}, {10,2}, {12,2}, {14,2},
#if RPI_PWM_NUM_SLICES > 8
     {16,2}, {18,2}, {20,2}, {22,2},
#endif
    }
};
static_assert(io_timers[0].base == RPI_PWM_BASE, "slice 0");
static_assert(io_timers[7].base == RPI_PWM_BASE + 7 * 0x14, "slice 7");
#if RPI_PWM_NUM_SLICES > 8
static_assert(io_timers[8].base == RPI_PWM_BASE + 0xa0, "slice 8");
static_assert(io_timers[11].base == RPI_PWM_BASE + 0xdc, "slice 11");
#endif
// Pins 16-29 reuse slices 0-6 (RP2040 datasheet 4.5.2): GPIO28 is slice 6 channel A
constexpr auto wrapped = initIOTimerChannel(io_timers, {Timer::Timer6, Timer::ChannelA}, {GPIO::Pin28});
static_assert(wrapped.timer_index == 6 && wrapped.timer_channel == 0, "pin 28 wraps to slice 6A");

constexpr unsigned LAST = MAX_IO_TIMERS - 1;              // last slice
constexpr unsigned LAST_A = 2 * LAST, LAST_B = 2 * LAST + 1; // its channels
constexpr unsigned ALL_CHANNELS = (1u << MAX_TIMER_IO_CHANNELS) - 1;

static uint32_t functions[64];
static bool outputs[64], levels[64];
extern "C" {
void rpi_gpio_set_pulls(uint32_t, int, int) {}
void rpi_gpio_setdir(uint32_t p, int out) { outputs[p] = out; }
void rpi_gpio_put(uint32_t p, int level) { levels[p] = level; }
void rpi_gpio_set_function(uint32_t p, uint32_t f) { functions[p] = f; }
int rpi_gpio_irq_attach(uint32_t, int, xcpt_t, void *) { return 0; }
void rpi_gpio_enable_irq(uint32_t) {}
void rpi_gpio_disable_irq(uint32_t) {}
}
static volatile uint32_t &reg(unsigned timer, unsigned offset) {
    return *reinterpret_cast<volatile uint32_t *>(io_timers[timer].base + offset);
}
static void map_page(uintptr_t base) {
    assert(mmap(reinterpret_cast<void *>(base & ~uintptr_t(4095)), 4096, PROT_READ | PROT_WRITE,
                MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0) != MAP_FAILED);
}
int main() {
    test_rom_header();
    map_page(RPI_PWM_BASE);
    const unsigned a = LAST_A, b = LAST_B, pin_a = timer_io_channels[a].gpio_out & GPIO_NUM_MASK,
                   pin_b = timer_io_channels[b].gpio_out & GPIO_NUM_MASK;
    assert(up_pwm_servo_init(ALL_CHANNELS) == (int)ALL_CHANNELS);
    assert(up_pwm_servo_set_rate(0) == -ENOTSUP);
    assert(up_pwm_servo_set_rate_group_update(LAST, 0) == -ENOTSUP);
    assert(io_timer_set_rate(LAST, 400) == 0);
    assert(io_timer_init_timer(LAST) == -EBUSY);
    assert(reg(LAST, RPI_PWM_TOP_OFFSET(0)) == 2499); // sibling initialization must not reset it
    assert(reg(LAST, RPI_PWM_DIV_OFFSET(0)) == ((BOARD_SYS_FREQ / 1000000u) << RPI_PWM_DIV_INT_SHIFT)); // 8.4 fixed point
    assert(up_pwm_servo_set(b, 1500) == 0 && up_pwm_servo_set(a, 1000) == 0);
    assert(reg(LAST, RPI_PWM_CC_OFFSET(0)) == ((1500u << RPI_PWM_CC_B_SHIFT) | 1000u));
    assert(up_pwm_servo_get(b) == 1500 && up_pwm_servo_get(a) == 1000);
    assert(io_timer_set_rate(LAST, 0) == -ENOTSUP);
    assert(reg(LAST, RPI_PWM_TOP_OFFSET(0)) == 2499);
    assert(io_timer_channel_init(0, IOTimerChanMode_OneShot, nullptr, nullptr) == -ENOTSUP);
    assert(io_timer_set_enable(true, IOTimerChanMode_PWMOut, (1u << a) | (1u << b)) == 0);
    assert(reg(LAST, 0) & RPI_PWM_CSR_EN);
    assert(io_timer_set_enable(false, IOTimerChanMode_PWMOut, 1u << a) == 0);
    assert(reg(LAST, 0) & RPI_PWM_CSR_EN); // B remains enabled
    assert(functions[pin_a] == RPI_GPIO_FUNC_SIO && outputs[pin_a] && !levels[pin_a]);
    assert(functions[pin_b] == RPI_GPIO_FUNC_PWM);
    assert(io_timer_free_channel(b) == 0);
    assert(!(reg(LAST, 0) & RPI_PWM_CSR_EN));
    assert(functions[pin_b] == RPI_GPIO_FUNC_SIO && outputs[pin_b] && !levels[pin_b]);
    assert(io_timer_set_enable(true, IOTimerChanMode_PWMOut, 1u << a) == 0);
    assert(functions[pin_a] == RPI_GPIO_FUNC_PWM);
    up_pwm_servo_deinit(0);
    assert(!(reg(LAST, 0) & RPI_PWM_CSR_EN));
    assert(io_timer_init_timer(MAX_IO_TIMERS) == -EINVAL);
    assert(io_timer_set_rate(MAX_IO_TIMERS, 50) == -EINVAL);
    assert(io_timer_set_rate(0, 1) == -ERANGE);

    assert(rpi_gpioconfig(cs0.cs_gpio) == 0);
    assert(functions[0] == RPI_GPIO_FUNC_SIO && outputs[0] && levels[0]);
    assert(rpi_gpioconfig(PX4_GPIO_PIN_OFF(cs0.cs_gpio)) == 0);
    assert(rpi_gpioconfig(cs0.cs_gpio) == 0); // restore CS after sensor reset
    assert(functions[0] == RPI_GPIO_FUNC_SIO && outputs[0] && levels[0]);
    assert(rpi_gpioconfig(PX4_MAKE_GPIO_INPUT(RPI_GPIO_NUM)) == -EINVAL); // first pin the chip does not have
    assert(rpi_gpioconfig(PX4_MAKE_GPIO_INPUT((RPI_GPIO_NUM - 1))) == 0);

    map_page(RPI_SYSINFO_BASE);
    auto chip_id = reinterpret_cast<volatile uint32_t *>(RPI_SYSINFO_BASE);
    *chip_id = TEST_CHIP_ID;
    char rev = 0; const char *revstr = nullptr; const char *errata = (const char *)1;
    assert(board_mcu_version(&rev, &revstr, &errata) == (int)(TEST_CHIP_ID >> 28));
    assert(rev == TEST_REV && std::strcmp(revstr, RPI_CHIP_NAME) == 0 && errata == nullptr);
    *chip_id = TEST_CHIP_ID ^ (1u << 12); // other part number
    assert(board_mcu_version(&rev, &revstr, &errata) == -1);
    *chip_id = TEST_CHIP_ID ^ 1u; // other manufacturer
    assert(board_mcu_version(&rev, &revstr, &errata) == -1);

    uuid_uint32_t uuid;
#if defined(CONFIG_ARCH_CHIP_RP23XX)
    map_page(RP23XX_OTP_DATA_BASE);
    auto otp = reinterpret_cast<uint32_t *>(RP23XX_OTP_DATA_BASE);
    otp[0] = 0x01234567; otp[1] = 0x89abcdef; otp[2] = 0xdeadbeef;
    board_get_uuid32(uuid);
    assert(uuid[0] == otp[0] && uuid[1] == otp[1] && uuid[2] == 0);
    const uint8_t expected[] = {0,0,0,0,0x89,0xab,0xcd,0xef,0x01,0x23,0x45,0x67};
#else
    board_get_uuid32(uuid);
    assert(std::memcmp(uuid, "PIPICORP2040", 12) == 0);
    const uint8_t expected[] = {'0','4','0','2','P','R','O','C','I','P','I','P'};
#endif
    mfguid_t mfg;
    px4_guid_t guid;
    board_get_mfguid(mfg);
    board_get_px4_guid(guid);
    assert(std::memcmp(mfg, expected, sizeof(expected)) == 0);
    assert(std::memcmp(guid + 6, expected, sizeof(expected)) == 0);
}
'''

SOURCES = ("io_pins/io_timer.c", "io_pins/pwm_servo.c", "io_pins/rpi_pinset.c",
           "version/board_identity.c", "version/board_mcu_version.c")


class PlatformTest(unittest.TestCase):
    def run_chip(self, chip):
        c = CHIPS[chip]
        with tempfile.TemporaryDirectory(prefix=f"{chip}-test-") as temp:
            temp = pathlib.Path(temp)
            for name, text in stubs(chip).items():
                path = temp / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
            includes = [temp, PLATFORM.parent / chip / "include",
                        ROOT / "platforms/common/include",
                        ROOT / "platforms/nuttx/src/px4/common/include",
                        NUTTX_ARCH / c["nuttx_dir"]]
            flags = [f"-I{p}" for p in includes]
            flags += [f"-DTEST_CHIP_ID={c['chip_id']:#x}u", f"-DTEST_REV='{c['rev']}'"]
            objects = []
            for source in SOURCES:
                obj = temp / (pathlib.Path(source).stem + ".o")
                subprocess.run(["cc", "-std=gnu11", "-Wall", "-Werror=implicit-function-declaration",
                                "-Wno-int-to-pointer-cast", *flags, "-c", str(PLATFORM / source),
                                "-o", str(obj)], check=True)
                objects.append(str(obj))
            test = temp / "test.cpp"
            test.write_text(TEST_CPP)
            exe = temp / "test"
            subprocess.run(["c++", "-std=c++14", "-Wall", *flags, str(test), *objects, "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_rp2040(self):
        self.run_chip("rp2040")

    def test_rp2350(self):
        self.run_chip("rp2350")


if __name__ == "__main__":
    unittest.main()
