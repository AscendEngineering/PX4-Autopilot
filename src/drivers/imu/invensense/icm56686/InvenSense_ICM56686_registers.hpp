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
 * @file InvenSense_ICM56686_registers.hpp
 *
 * Invensense ICM-56686 registers.
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace InvenSense_ICM56686
{
// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint32_t SPI_SPEED = 24 * 1000 * 1000; // 24 MHz SPI
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHOAMI = 0x08;
static constexpr uint8_t FIFO_CONFIG2_RESET = 0x20;
static constexpr float TEMPERATURE_SENSITIVITY = 128.f; // LSB/C
static constexpr float TEMPERATURE_OFFSET = 25.f; // C

namespace Register
{

enum class BANK_0 : uint8_t {
	TEMP_DATA1_UI = 0x0c,
	TEMP_DATA0_UI = 0x0d,

	PWR_MGMT0 = 0x14,
	FIFO_COUNT_0 = 0x16,
	FIFO_COUNT_1 = 0x17,
	FIFO_DATA = 0x18,
	INT1_CONFIG0 = 0x1A,
	INT1_CONFIG1 = 0x1B,
	INT1_CONFIG2 = 0x1C,
	INT1_STATUS0 = 0x1D,
	ACCEL_CONFIG0 = 0x1F,
	GYRO_CONFIG0 = 0x20,
	FIFO_CONFIG0 = 0x21,
	FIFO_CONFIG1_0 = 0x22,
	FIFO_CONFIG1_1 = 0x23,
	FIFO_CONFIG2 = 0x24,
	FIFO_CONFIG3 = 0x25,
	FIFO_CONFIG4 = 0x26,
	RTC_CONFIG = 0x2A,
	DMP_EXT_SEN_ODR_CFG = 0x2B,
	EDMP_APEX_EN0 = 0x2D,
	EDMP_APEX_EN1 = 0x2E,
	APEX_BUFFER_MGMT = 0x2F,
	INTF_CONFIG0 = 0x30,
	INTF_CONFIG1_OVRD = 0x31,
	INTF_AUX_CONFIG = 0x32,
	IOC_PAD_SCENARIO = 0x33,
	IOC_PAD_SCENARIO_AUX_OVRD = 0x34,
	IOC_PAD_SCENARIO_OVRD = 0x35,
	DRIVE_CONFIG0 = 0x36,
	DRIVE_CONFIG1 = 0x37,
	DRIVE_CONFIG2 = 0x38,
	INT_APEX_CONFIG1 = 0x3E,
	INT_APEX_STATUS0 = 0x3F,
	INT_APEX_STATUS1 = 0x40,
	PWR_MGMT_AUX1 = 0x58,
	INT2_CONFIG0 = 0x5A,
	INT2_CONFIG1 = 0x60,
	INT2_CONFIG2 = 0x61,
	INT2_STATUS0 = 0x62,
	WHO_AM_I = 0x72,
	IREG_ADDR_15_8 = 0x7C,
	IREG_ADDR_7_0 = 0x7D,
	IREG_DATA = 0x7E,
	REG_MISC2 = 0x7F,
};

// Indirect registers, reached through IREG_ADDR_15_8/IREG_ADDR_7_0 + IREG_DATA as {bank base | offset}
enum class IREG : uint16_t {
	IPREG_SYS1_REG_166 = 0xA400 | 0x9A, // GYRO_SRC_CTRL
	IPREG_SYS2_REG_123 = 0xA500 | 0x6D, // ACCEL_SRC_CTRL
};

};

//---------------- BANK0 Register bits

// PWR_MGMT0
enum PWR_MGMT0_BIT : uint8_t {
	GYRO_MODE_LOW_NOISE  = Bit3 | Bit2, // 11: Places gyroscope in Low Noise (LN) Mode
	ACCEL_MODE_LOW_NOISE = Bit1 | Bit0, // 11: Places accelerometer in Low Noise (LN) Mode
};

enum PWR_MGMT_AUX1_BIT : uint8_t {
	AUX1_DISABLE  = Bit1 | Bit0, // 3: Turn Off AUX1
};
enum INT1_STATUS0 : uint8_t {
	INT1_STATUS_RESET_DONE = Bit7,
	INT1_STATUS_AUX1_AGC = Bit6,
	INT1_STATUS_AP_AGC_RDY = Bit5,
	INT1_STATUS_AP_FSYNC = Bit4,
	INT1_STATUS_AP_AUX1_DRDY = Bit3,
	INT1_STATUS_AP_DRDY = Bit2,
	INT1_STATUS_FIFO_THS = Bit1,
	INT1_STATUS_FIFO_FULL = Bit0,
};

// INT1_CONFIG0: routes the corresponding status (same bit positions as INT1_STATUS0) to INT1
enum INT1_CONFIG0_BIT : uint8_t {
	INT1_STATUS_EN_RESET_DONE = Bit7,
	INT1_STATUS_EN_AUX1_AGC = Bit6,
	INT1_STATUS_EN_AP_AGC_RDY = Bit5,
	INT1_STATUS_EN_AP_FSYNC = Bit4,
	INT1_STATUS_EN_AP_AUX1_DRDY = Bit3,
	INT1_STATUS_EN_AP_DRDY = Bit2,
	INT1_STATUS_EN_FIFO_THS = Bit1,
	INT1_STATUS_EN_FIFO_FULL = Bit0,
};

// INT1_CONFIG2
enum INT1_CONFIG2_BIT : uint8_t {
	INT1_DRIVE    = Bit2, // 0: push-pull, 1: open drain
	INT1_MODE     = Bit1, // 0: pulsed, 1: latched
	INT1_POLARITY = Bit0, // 0: active low, 1: active high
};

enum ACCEL_CONFIG0_BIT : uint8_t {
	ACCEL_UI_FS_SEL_32_G_SET = 0,
	ACCEL_UI_FS_SEL_32_G_CLEAR = Bit6 | Bit5 | Bit4,
	ACCEL_UI_FS_SEL_16_G_SET = Bit4,
	ACCEL_UI_FS_SEL_16_G_CLEAR = Bit6 | Bit5,
	ACCEL_UI_FS_SEL_8_G_SET = Bit5,
	ACCEL_UI_FS_SEL_8_G_CLEAR = Bit6 | Bit4,
	ACCEL_ODR_6400_HZ_SET = Bit0 | Bit1,
	ACCEL_ODR_6400_HZ_CLEAR = Bit2,
	ACCEL_ODR_3200_HZ_SET = Bit2,
	ACCEL_ODR_3200_HZ_CLEAR = Bit0 | Bit1,
	ACCEL_ODR_1600_HZ_SET = Bit2 | Bit0,
	ACCEL_ODR_1600_HZ_CLEAR = Bit1,
	ACCEL_ODR_800_HZ_SET = Bit2 | Bit1,
	ACCEL_ODR_800_HZ_CLEAR = Bit0,
};

enum GYRO_CONFIG0_BIT : uint8_t {
	GYRO_UI_FS_SEL_4000_DPS_SET = 0,
	GYRO_UI_FS_SEL_4000_DPS_CLEAR = Bit7 | Bit6 | Bit5 | Bit4,
	GYRO_UI_FS_SEL_2000_DPS_SET = Bit4,
	GYRO_UI_FS_SEL_2000_DPS_CLEAR = Bit7 | Bit6 | Bit5,
	GYRO_UI_FS_SEL_1000_DPS_SET = Bit5,
	GYRO_UI_FS_SEL_1000_DPS_CLEAR = Bit7 | Bit6 | Bit4,
	GYRO_ODR_6400_HZ_SET = Bit0 | Bit1,
	GYRO_ODR_6400_HZ_CLEAR = Bit2,
	GYRO_ODR_3200_HZ_SET = Bit2,
	GYRO_ODR_3200_HZ_CLEAR = Bit0 | Bit1,
	GYRO_ODR_1600_HZ_SET = Bit2 | Bit0,
	GYRO_ODR_1600_HZ_CLEAR = Bit1,
	GYRO_ODR_800_HZ_SET = Bit2 | Bit1,
	GYRO_ODR_800_HZ_CLEAR = Bit0,
};

enum FIFO_CONFIG0_BIT : uint8_t {
	FIFO_MODE_BYPASS_SET = 0,
	FIFO_MODE_BYPASS_CLEAR = Bit0 | Bit1 | Bit2,
	FIFO_MODE_STREAM_SET = Bit6,
	FIFO_MODE_STREAM_CLEAR = Bit7,
	FIFO_MODE_STOP_ON_FULL_SET = Bit7,
	FIFO_MODE_STOP_ON_FULL_CLEAR = Bit6,
	FIFO_DEPTH_2K_SET = Bit0 | Bit1 | Bit2,
	FIFO_DEPTH_2K_CLEAR = Bit3 | Bit4,
	FIFO_DEPTH_8K_SET = Bit0 | Bit1 | Bit2 | Bit3 | Bit4,
	FIFO_DEPTH_8K_CLEAR = 0,
};

enum FIFO_CONFIG2_BIT : uint8_t {
	FIFO_FLUSH = Bit7,
	FIFO_WR_WM_EQ_TH = 0,        // watermark reached only when count == threshold
	FIFO_WR_WM_EQ_OR_GT_TH = Bit3, // watermark reached when count >= threshold
};

enum FIFO_CONFIG3_BIT : uint8_t {
	FIFO_ES1_EN = Bit5, // External sensor 1 data insertion into FIFO frame
	FIFO_ES0_EN = Bit4, // External sensor 0 data insertion into FIFO frame
	FIFO_HIRES_EN = Bit3, // High resolution accel and gyro data insertion into FIFO frame
	FIFO_GYRO_EN = Bit2, // Gyro data insertion into FIFO frame
	FIFO_ACCEL_EN = Bit1, // Accel data insertion into FIFO frame
	FIFO_IF_EN = Bit0, // Enable FIFO
};

enum FIFO_CONFIG4_BIT : uint8_t {
	FIFO_COMP_EN = Bit2, // FIFO compression enabled
	FIFO_TMST_FSYNC_EN = Bit1, // Timestamp/FSYNC data inserted into FIFO frame
};

enum RTC_CONFIG_BIT : uint8_t {
	RTC_ALIGN = Bit6, // Re-align command is generated by writing 1 to this bit
	RTC_MODE = Bit5, // 0: RTC functionality not enabled, 1: RTC functionality enabled
};

enum IOC_PAD_SCENARIO_OVRD_BIT : uint8_t {
	PADS_INT2_CFG_OVRD = Bit2, // Override enable for PADS_INT2_CFG, 0: disable, 1: enable
	PADS_INT2_CFG_OVRD_INT2 = 0,
	PADS_INT2_CFG_OVRD_FSYNC = Bit0,
	PADS_INT2_CFG_OVRD_CLKIN = Bit1,
};

enum REG_MISC2_BIT : uint8_t {
	SOFT_RST = Bit1, // 1: Triggers soft reset operation
};


//---------------- IREG bits

// IPREG_SYS1_REG_166
enum IPREG_SYS1_REG_166_BIT : uint8_t {
	// GYRO_SRC_CTRL[3:2] 10: interpolator and FIR anti-alias filter on (reset 00: both off)
	GYRO_SRC_CTRL_INTERP_AAF_SET   = Bit3,
	GYRO_SRC_CTRL_INTERP_AAF_CLEAR = Bit2,
};

// IPREG_SYS2_REG_123
enum IPREG_SYS2_REG_123_BIT : uint8_t {
	// ACCEL_SRC_CTRL[1:0] 10: interpolator and FIR anti-alias filter on (reset 00: both off)
	ACCEL_SRC_CTRL_INTERP_AAF_SET   = Bit1,
	ACCEL_SRC_CTRL_INTERP_AAF_CLEAR = Bit0,
};

// IPREG_TOP1
static constexpr uint8_t BANK_IPREG_TOP1 = 0xA2;
static constexpr uint8_t SREG_CTRL = 0x60;
enum SREG_CTRL_SREG_DATA_ENDIAN_SEL_BIT : uint8_t {
  SREG_CTRL_SREG_DATA_ENDIAN_SEL_BIG = Bit1, // big endian as documented (instead of default little endian)
};


namespace FIFO
{
static constexpr size_t SIZE = 8192;

// 16-byte packet (accel + gyro, FIFO_HIRES_EN clear)
struct DATA {
	uint8_t FIFO_Header;
	uint8_t ACCEL_DATA_XH;
	uint8_t ACCEL_DATA_XL;
	uint8_t ACCEL_DATA_YH;
	uint8_t ACCEL_DATA_YL;
	uint8_t ACCEL_DATA_ZH;
	uint8_t ACCEL_DATA_ZL;
	uint8_t GYRO_DATA_XH;
	uint8_t GYRO_DATA_XL;
	uint8_t GYRO_DATA_YH;
	uint8_t GYRO_DATA_YL;
	uint8_t GYRO_DATA_ZH;
	uint8_t GYRO_DATA_ZL;
	uint8_t temperature;
	uint8_t Timestamp_H;
	uint8_t Timestamp_L;
};
static_assert(sizeof(DATA) == 16, "FIFO packet is 16 bytes without HIRES");

// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
enum FIFO_HEADER_BIT : uint8_t {
	HEADER_MSG             = Bit7, // 1: FIFO is empty
	HEADER_ACCEL           = Bit6, // 1: Packet is sized so that accel data have location in the packet, FIFO_ACCEL_EN must be 1
	HEADER_GYRO            = Bit5, // 1: Packet is sized so that gyro data have location in the packet, FIFO_GYRO_EN must be1
	HEADER_20              = Bit4, // 1: Packet has a new and valid sample of extended 20-bit data for gyro and/or accel
	HEADER_TIMESTAMP_FSYNC = Bit3 | Bit2, // 10: Packet contains ODR Timestamp
	HEADER_ODR_ACCEL       = Bit1, // 1: The ODR for accel is different for this accel data packet compared to the previous accel packet
	HEADER_ODR_GYRO        = Bit0, // 1: The ODR for gyro is different for this gyro data packet compared to the previous gyro packet
};

}
} // namespace InvenSense_ICM56686
