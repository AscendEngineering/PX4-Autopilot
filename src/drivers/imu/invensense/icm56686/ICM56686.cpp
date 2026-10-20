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

#include "ICM56686.hpp"

using namespace time_literals;

// Diagnostics
// Removed diagnostics

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

static constexpr uint16_t combine_uint(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM56686::ICM56686(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	if (config.drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	// Force internal oscillator use. External clock config (custom1) is ignored
	// because the rpi-uavfc board does not currently generate a clock on GPIO 27.
	_enable_clock_input = false;

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ICM56686::~ICM56686()
{
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int ICM56686::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM56686::Reset()
{
	DataReadyInterruptDisable();
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM56686::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM56686::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);
	PX4_INFO("Clock input: %s", _enable_clock_input ? "enabled" : "disabled");

	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int ICM56686::probe()
{
	for (int i = 0; i < 3; i++) {
		const uint8_t whoami = RegisterRead(Register::BANK_0::WHO_AM_I);

		if (whoami == WHOAMI) {
			return PX4_OK;
		}

		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
	}

	return PX4_ERROR;
}

void ICM56686::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// Bypassing SOFT_RST to align with ArduPilot initialization and avoid wiping config.
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::BANK_0::WHO_AM_I) == WHOAMI)) {

			RegisterWrite(Register::BANK_0::PWR_MGMT_AUX1, PWR_MGMT_AUX1_BIT::AUX1_DISABLE);

			// Wakeup accel and gyro
			RegisterWrite(Register::BANK_0::PWR_MGMT0, PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE);

			_state = STATE::CONFIGURE;
			ScheduleDelayed(100_ms); // Increased delay to ensure analog front-end stability

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then reset the FIFO
			_state = STATE::FIFO_RESET;
			ScheduleDelayed(1_ms);

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_RESET:

		_state = STATE::FIFO_READ;
		FIFOReset();

		if (DataReadyInterruptConfigure()) {
			_data_ready_interrupt_enabled = true;

			// backup schedule as a watchdog timeout
			ScheduleDelayed(100_ms);

		} else {
			_data_ready_interrupt_enabled = false;
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			bool success = false;

			if (FIFORead(timestamp_sample)) {
				success = true;

				if (_failure_count > 0) {
					_failure_count--;
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}

			} else if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
				UpdateTemperature();
				_temperature_update_timestamp = now;
			}
		}

		break;
	}
}

void ICM56686::ConfigureSampleRate(int sample_rate)
{
	// round down to the nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void ICM56686::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold is a FIFO frame count, unlike the ICM42688P where it counts bytes
	const uint16_t fifo_watermark_threshold = samples;

	for (auto &r : _register_bank0_cfg) {
		if (r.reg == Register::BANK_0::FIFO_CONFIG1_0) {
			// FIFO_WM[7:0]
			const uint8_t value = fifo_watermark_threshold & 0xFF;
			r.set_bits = value;
			r.clear_bits = static_cast<uint8_t>(~value);

		} else if (r.reg == Register::BANK_0::FIFO_CONFIG1_1) {
			// FIFO_WM[15:8]
			const uint8_t value = (fifo_watermark_threshold >> 8) & 0xFF;
			r.set_bits = value;
			r.clear_bits = static_cast<uint8_t>(~value);
		}
	}
}

void ICM56686::ConfigureCLKIN()
{
	for (auto &r0 : _register_bank0_cfg) {
		if (r0.reg == Register::BANK_0::RTC_CONFIG) {
			r0.set_bits = RTC_CONFIG_BIT::RTC_MODE;
		}
	}

	for (auto &r0 : _register_bank0_cfg) {
		if (r0.reg == Register::BANK_0::IOC_PAD_SCENARIO_OVRD) {
			r0.set_bits = PADS_INT2_CFG_OVRD | PADS_INT2_CFG_OVRD_CLKIN;
		}
	}
}

bool ICM56686::Configure()
{
	// Set it to little endian first.
	// On some versions of this chip, SREG_CTRL must be written with specific timing.
	// uint8_t cmd[3] {
	//    BANK_IPREG_TOP1,
	//        SREG_CTRL,
	//    SREG_CTRL_SREG_DATA_ENDIAN_SEL_BIT::SREG_CTRL_SREG_DATA_ENDIAN_SEL_BIG };
	// transfer(cmd, cmd, sizeof(cmd));
	// px4_udelay(100);

	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_ireg_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// Disable FIFO.
	RegisterClearBits(Register::BANK_0::FIFO_CONFIG3,
			  FIFO_CONFIG3_BIT::FIFO_GYRO_EN | FIFO_CONFIG3_BIT::FIFO_ACCEL_EN | FIFO_CONFIG3_BIT::FIFO_IF_EN);
	// Enable interpolator and FIR anti-alias filter (ArduPilot reference)
	RegisterWrite(Register::IREG::IPREG_SYS1_REG_166, 0x0C); // GYRO_SRC_CTRL bits[3:2] = 11 (Both on)
	RegisterWrite(Register::IREG::IPREG_SYS2_REG_123, 0x03); // ACCEL_SRC_CTRL bits[1:0] = 11 (Both on)

	// Explicitly ensure we are using the internal oscillator if external clock is disabled.
	if (!_enable_clock_input) {
		RegisterClearBits(Register::BANK_0::RTC_CONFIG, RTC_CONFIG_BIT::RTC_MODE);
		RegisterClearBits(Register::BANK_0::IOC_PAD_SCENARIO_OVRD, PADS_INT2_CFG_OVRD | PADS_INT2_CFG_OVRD_CLKIN);
	}

	// FIFO flush and reset: trigger flush and reset.
	// FIFO_FLUSH is auto-cleared by the hardware, but must be triggered for the FIFO
	// to accept configuration.
	RegisterWrite(Register::BANK_0::FIFO_CONFIG2, FIFO_CONFIG2_BIT::FIFO_FLUSH | FIFO_CONFIG2_BIT::FIFO_RESET);
	px4_udelay(100);

	// ArduPilot: FIFO_CONFIG2 reset keeps bit5 (fifo_addr_space_lock) set — value 0x20.
	RegisterWrite(Register::BANK_0::FIFO_CONFIG2, FIFO_CONFIG2_BIT::FIFO_RESET);

	RegisterSetBits(Register::BANK_0::FIFO_CONFIG3,
			FIFO_CONFIG3_BIT::FIFO_GYRO_EN |
			FIFO_CONFIG3_BIT::FIFO_ACCEL_EN |
			FIFO_CONFIG3_BIT::FIFO_IF_EN);

	// Force FIFO Interface enable.
	RegisterWrite(Register::BANK_0::FIFO_CONFIG3,
		      FIFO_CONFIG3_BIT::FIFO_GYRO_EN | FIFO_CONFIG3_BIT::FIFO_ACCEL_EN | FIFO_CONFIG3_BIT::FIFO_IF_EN);

	// Set depth 2k AND stop-on-full.
	// ArduPilot uses 0x87 (FIFO_DEPTH_2K_SET | FIFO_MODE_STOP_ON_FULL_SET).
	RegisterWrite(Register::BANK_0::FIFO_CONFIG0, FIFO_CONFIG0_BIT::FIFO_DEPTH_2K_SET | FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_SET);

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_ireg_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// 16-bit FIFO (FIFO_HIRES_EN clear). Full-scale is ±4000 dps / ±32 g.
	_px4_accel.set_range(32.f * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math::radians(4000.f));
	_px4_accel.set_scale(32.f * CONSTANTS_ONE_G / 32768.f);
	_px4_gyro.set_scale(math::radians(4000.f / 32768.f));

	return success;
}

template <typename T>
bool ICM56686::RegisterCheck(const T &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

template <typename T>
uint8_t ICM56686::RegisterRead(T reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

template <typename T>
void ICM56686::RegisterWrite(T reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

// IREG access (datasheet §14): address and data go in one burst starting at IREG_ADDR_15_8, and the
// part needs at least 4 us between consecutive IREG operations. A read is a pre-fetch triggered by
// CS deasserting after the address write, collected from IREG_DATA after the same gap.
uint8_t ICM56686::RegisterRead(Register::IREG reg)
{
	const uint16_t addr = static_cast<uint16_t>(reg);
	uint8_t cmd[3] { static_cast<uint8_t>(Register::BANK_0::IREG_ADDR_15_8), static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF) };
	transfer(cmd, cmd, sizeof(cmd));
	px4_udelay(10);
	const uint8_t value = RegisterRead(Register::BANK_0::IREG_DATA);
	px4_udelay(10);
	return value;
}

void ICM56686::RegisterWrite(Register::IREG reg, uint8_t value)
{
	const uint16_t addr = static_cast<uint16_t>(reg);
	uint8_t cmd[4] { static_cast<uint8_t>(Register::BANK_0::IREG_ADDR_15_8), static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF), value };
	transfer(cmd, cmd, sizeof(cmd));
	px4_udelay(10);
}

template <typename T>
void ICM56686::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

int ICM56686::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM56686 *>(arg)->DataReady();
	return 0;
}

void ICM56686::DataReady()
{
#if 0
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
#endif
}

bool ICM56686::DataReadyInterruptConfigure()
{
# if 0

	if (_drdy_gpio == 0) {
		return false;
	}

	// Enable internal pull-up to prevent the pin from floating low (open-drain)
	px4_arch_configgpio(_drdy_gpio | GPIO_PU_MASK);

	// Setup data ready on falling edge (INT1 is configured pulsed, active low)
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
#endif
	return false;
}

bool ICM56686::DataReadyInterruptDisable()
{
	// if (_drdy_gpio == 0) {
	return false;
	// }
	//
	// return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

uint16_t ICM56686::FIFOReadCount()
{
	// read FIFO count
	uint8_t fifo_count_buf[3] { static_cast<uint8_t>(Register::BANK_0::FIFO_COUNT_0) | DIR_READ, 0, 0 };

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	// FIFO_COUNT_0 is high bits, FIFO_COUNT_1 is low bits.
	// Since the device is configured for Little Endian, they are swapped.
	return combine_uint(fifo_count_buf[2], fifo_count_buf[1]);
}

bool ICM56686::FIFORead(const hrt_abstime &timestamp_sample)
{
	const uint16_t fifo_packets = FIFOReadCount();

	if (fifo_packets == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	if (fifo_packets > FIFO_MAX_SAMPLES) {
		// More frames queued than a single transfer can publish. Draining them would stamp stale
		// samples with the current timestamp, so reset for a clean restart instead. Also covers
		// the FIFO being saturated, where stop-on-full has already dropped newer samples.
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	const size_t transfer_size = math::min(sizeof(FIFOTransferBuffer), fifo_packets * sizeof(FIFO::DATA) + 1);

	// _fifo_buffer is reused across cycles. transfer() clobbers the command byte with the byte
	// clocked in alongside it, so restore it; the rest needs no clearing because only the frames
	// covered by transfer_size are ever parsed.
	_fifo_buffer.cmd = static_cast<uint8_t>(Register::BANK_0::FIFO_DATA) | DIR_READ;

	if (transfer((uint8_t *)&_fifo_buffer, (uint8_t *)&_fifo_buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	// Search for the first valid FIFO header to handle SPI alignment offsets.
	// Valid header for Accel+Gyro+Timestamp: (header & 0xFC) == 0x68 (std) or 0x78 (hi-res).
	int offset = -1;

	for (int i = 0; i < static_cast<int>(transfer_size) - static_cast<int>(sizeof(FIFO::DATA)); i++) {
		uint8_t header = ((uint8_t *)&_fifo_buffer)[i];

		if ((header & 0xFC) == 0x68 || (header & 0xFC) == 0x78) {
			offset = i;
			break;
		}
	}

	if (offset == -1) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	unsigned valid_samples = 0;
	const uint8_t *data_ptr = (uint8_t *)&_fifo_buffer + offset;

	for (unsigned i = 0; (offset + (i + 1) * sizeof(FIFO::DATA)) <= transfer_size; i++) {
		bool valid = true;
		const FIFO::DATA *sample = (const FIFO::DATA *)(data_ptr + (i * sizeof(FIFO::DATA)));
		const uint8_t FIFO_HEADER = sample->FIFO_Header;

		if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_MSG) {
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ACCEL)) {
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_GYRO)) {
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_20) {
			// Note: We handle 20-bit extension if needed, but for now 16-bit is expected
			valid = false;

		} else if ((FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_TIMESTAMP_FSYNC) != Bit3) {
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_ACCEL) {
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_GYRO) {
			valid = false;
		}

		if (valid) {
			valid_samples++;

		} else {
			perf_count(_bad_transfer_perf);
			break;
		}
	}

	if (valid_samples > 0) {
		// Pass the aligned data pointer to the processors
		const FIFO::DATA *aligned_fifo = (const FIFO::DATA *)data_ptr;
		ProcessGyro(timestamp_sample, aligned_fifo, valid_samples);
		ProcessAccel(timestamp_sample, aligned_fifo, valid_samples);
		return true;
	}

	return false;
}

void ICM56686::FIFOReset()
{
	perf_count(_fifo_reset_perf);
	_drdy_timestamp_sample.store(0);

	// FIFO_FLUSH is auto cleared. Keep fifo_addr_space_lock (bit5) set
	// reset value 0x20
	RegisterSetBits(Register::BANK_0::FIFO_CONFIG2,
			FIFO_CONFIG2_BIT::FIFO_RESET | FIFO_CONFIG2_BIT::FIFO_FLUSH);
	RegisterClearBits(Register::BANK_0::FIFO_CONFIG2,
			  FIFO_CONFIG2_BIT::FIFO_RESET);
}

void ICM56686::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		if (_enable_clock_input) {
			// Swapped as device is in little endian by default.
			const uint16_t timestamp_fifo = combine_uint(fifo[i].Timestamp_L, fifo[i].Timestamp_H);
			accel.dt = (float)timestamp_fifo * ((1.f / _input_clock_freq) * 1e6f);
		}

		const int16_t accel_x = combine(fifo[i].ACCEL_DATA_XL, fifo[i].ACCEL_DATA_XH);
		const int16_t accel_y = combine(fifo[i].ACCEL_DATA_YL, fifo[i].ACCEL_DATA_YH);
		const int16_t accel_z = combine(fifo[i].ACCEL_DATA_ZL, fifo[i].ACCEL_DATA_ZH);

		// sample invalid if -32768 (FIFO_HOLD_LAST_DATA_EN is clear)
		if (accel_x != INT16_MIN && accel_y != INT16_MIN && accel_z != INT16_MIN) {
			accel.x[accel.samples] = accel_x;
			accel.y[accel.samples] = accel_y;
			accel.z[accel.samples] = accel_z;
			accel.samples++;
		}
	}

	// correct frame for publication
	for (int i = 0; i < accel.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MAX : -accel.y[i];
		accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MAX : -accel.z[i];
	}

	_px4_accel.set_error_count(perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}
}

void ICM56686::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;
	gyro.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		if (_enable_clock_input) {
			// Swapped as device is in little endian by default.
			const uint16_t timestamp_fifo = combine_uint(fifo[i].Timestamp_L, fifo[i].Timestamp_H);
			gyro.dt = (float)timestamp_fifo * ((1.f / _input_clock_freq) * 1e6f);
		}

		const int16_t gyro_x = combine(fifo[i].GYRO_DATA_XL, fifo[i].GYRO_DATA_XH);
		const int16_t gyro_y = combine(fifo[i].GYRO_DATA_YL, fifo[i].GYRO_DATA_YH);
		const int16_t gyro_z = combine(fifo[i].GYRO_DATA_ZL, fifo[i].GYRO_DATA_ZH);

		// sample invalid if -32768 (FIFO_HOLD_LAST_DATA_EN is clear)
		if (gyro_x != INT16_MIN && gyro_y != INT16_MIN && gyro_z != INT16_MIN) {
			gyro.x[gyro.samples] = gyro_x;
			gyro.y[gyro.samples] = gyro_y;
			gyro.z[gyro.samples] = gyro_z;
			gyro.samples++;
		}
	}

	// correct frame for publication
	for (int i = 0; i < gyro.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MAX : -gyro.y[i];
		gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MAX : -gyro.z[i];
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (gyro.samples > 0) {
		_px4_gyro.updateFIFO(gyro);
	}
}

void ICM56686::UpdateTemperature()
{
	uint8_t temperature_buf[3] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1_UI) | DIR_READ;

	if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const int16_t TEMP_DATA = combine(temperature_buf[2], temperature_buf[1]);
	const float temp_c = (TEMP_DATA / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(temp_c)) {
		_px4_accel.set_temperature(temp_c);
		_px4_gyro.set_temperature(temp_c);
	}
}
