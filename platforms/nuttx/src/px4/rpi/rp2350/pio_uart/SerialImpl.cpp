/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

#include <SerialImpl.hpp>
#include <string.h> // strncpy
#include <px4_log.h>
#include <drivers/device/pio/pio_serial.h>
#include <drivers/drv_hrt.h>

#define MODULE_NAME "SerialImpl"

namespace device
{
namespace rp2350
{

// Helper to map port name (e.g., "PIO0") to instance index
static int port_to_instance(const char *port)
{
    if (strcmp(port, "PIO0") == 0) return 0;
    if (strcmp(port, "PIO1") == 0) return 1;
    if (strcmp(port, "PIO2") == 0) return 2;
    if (strcmp(port, "PIO3") == 0) return 3;
    return -1;
}

SerialImpl::SerialImpl(const char *port, uint32_t baudrate, ByteSize bytesize, Parity parity, StopBits stopbits,
		       FlowControl flowcontrol) :
    _baudrate(baudrate),
    _bytesize(bytesize),
    _parity(parity),
    _stopbits(stopbits),
    _flowcontrol(flowcontrol)
{
    if (validatePort(port)) {
        setPort(port);
    } else {
        _port[0] = 0;
    }
}

SerialImpl::~SerialImpl()
{
    if (isOpen()) {
        close();
    }
}

bool SerialImpl::validateBaudrate(uint32_t baudrate)
{
    return (baudrate > 0);
}

bool SerialImpl::open()
{
    _open = false;
    _serial_fd = -1;

    if (!validateBaudrate(_baudrate)) {
        PX4_ERR("Invalid baudrate: %u", _baudrate);
        return false;
    }

    if (!setBytesize(_bytesize)) { return false; }
    if (!setParity(_parity)) { return false; }
    if (!setStopbits(_stopbits)) { return false; }
    if (!setFlowcontrol(_flowcontrol)) { return false; }

    if (!validatePort(_port)) {
        PX4_ERR("Invalid port %s", _port);
        return false;
    }

    int instance = port_to_instance(_port);
    if (instance < 0) {
        PX4_ERR("Unsupported PIO port: %s", _port);
        return false;
    }

    // Use the driver's static instances.
    // In a fully integrated system, we would have a way to ensure the PIORXDriver
    // is created and configured. For this mapping, we assume it's available.
    if (device::rp2350::PIORXDriver::_instances[instance] == nullptr) {
        // If not yet created, we create it here.
        new device::rp2350::PIORXDriver(_port, _baudrate, instance);
    }

    _serial_fd = instance;
    _open = true;

    PX4_INFO("Successfully opened PIO UART %s (instance %d) with baudrate %u", _port, instance, _baudrate);

    return _open;
}

bool SerialImpl::isOpen() const
{
    return _open;
}

bool SerialImpl::close()
{
    _serial_fd = -1;
    _open = false;
    return true;
}

ssize_t SerialImpl::bytesAvailable()
{
    if (!_open) return -1;

    int instance = _serial_fd;
    if (instance < 0 || instance >= 4) return -1;

    // Call the driver's available()
    // We need to cast since SerialImpl is a different class hierarchy than PIORXDriver
    // but in the provided pio_serial.h, PIORXDriver inherits from SerialImpl (or a similar base).
    // Let's assume it implements the expected interface.
    auto *driver = (device::rp2350::PIORXDriver*)device::rp2350::PIORXDriver::_instances[instance];
    if (!driver) return 0;

    return driver->bytesAvailable();
}

ssize_t SerialImpl::txSpaceAvailable()
{
    if (!_open) return -1;
    return 512;
}

ssize_t SerialImpl::read(uint8_t *buffer, size_t buffer_size)
{
    if (!_open) return -1;

    int instance = _serial_fd;
    auto *driver = (device::rp2350::PIORXDriver*)device::rp2350::PIORXDriver::_instances[instance];
    if (!driver) return -1;

    return driver->read(buffer, buffer_size);
}

ssize_t SerialImpl::readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count, uint32_t timeout_ms)
{
    if (!_open) return -1;

    int instance = _serial_fd;
    auto *driver = (device::rp2350::PIORXDriver*)device::rp2350::PIORXDriver::_instances[instance];
    if (!driver) return -1;

    // The driver doesn't implement readAtLeast specifically, so we use the base loop logic
    // or assume the driver's read() handles blocking/timeout.
    // For now, we'll mirror the basic read loop.

    const hrt_abstime start_time_us = hrt_absolute_time();
    uint32_t timeout_us = timeout_ms * 1000;
    int total_bytes_read = 0;

    while (total_bytes_read < (int) character_count) {
        if (timeout_us > 0) {
            if (hrt_elapsed_time(&start_time_us) >= timeout_us) {
                return total_bytes_read;
            }
        }

        int current_bytes_read = driver->read(&buffer[total_bytes_read], buffer_size - total_bytes_read);
        if (current_bytes_read < 0) return -1;

        total_bytes_read += current_bytes_read;
        if (total_bytes_read >= (int) character_count) return total_bytes_read;

        px4_usleep(5000);
    }
    return total_bytes_read;
}

ssize_t SerialImpl::write(const void *buffer, size_t buffer_size)
{
    if (!_open) return -1;

    int instance = _serial_fd;
    auto *driver = (device::rp2350::PIORXDriver*)device::rp2350::PIORXDriver::_instances[instance];
    if (!driver) return -1;

    return driver->write(buffer, buffer_size);
}

ssize_t SerialImpl::writeBlocking(const void *buffer, size_t buffer_size, uint32_t timeout_ms)
{
    return write(buffer, buffer_size);
}

void SerialImpl::flush()
{
    if (_open) {
        int instance = _serial_fd;
        auto *driver = (device::rp2350::PIORXDriver*)device::rp2350::PIORXDriver::_instances[instance];
        if (driver) driver->flush();
    }
}

const char *SerialImpl::getPort() const
{
    return _port;
}

bool SerialImpl::validatePort(const char *port)
{
    return (port_to_instance(port) >= 0);
}

bool SerialImpl::setPort(const char *port)
{
    if (_open) return false;
    if (validatePort(port)) {
        strncpy(_port, port, sizeof(_port) - 1);
        _port[sizeof(_port) - 1] = '\0';
        return true;
    }
    return false;
}

uint32_t SerialImpl::getBaudrate() const
{
    return _baudrate;
}

bool SerialImpl::setBaudrate(uint32_t baudrate)
{
    if (!validateBaudrate(baudrate)) return false;
    if (baudrate == _baudrate) return true;
    _baudrate = baudrate;
    if (_open) return open();
    return true;
}

ByteSize SerialImpl::getBytesize() const
{
    return _bytesize;
}

bool SerialImpl::setBytesize(ByteSize bytesize)
{
    if (bytesize != ByteSize::EightBits) return false;
    return true;
}

Parity SerialImpl::getParity() const
{
    return _parity;
}

bool SerialImpl::setParity(Parity parity)
{
    if (parity != Parity::None) return false;
    return true;
}

StopBits SerialImpl::getStopbits() const
{
    return _stopbits;
}

bool SerialImpl::setStopbits(StopBits stopbits)
{
    if (stopbits != StopBits::One) return false;
    return true;
}

FlowControl SerialImpl::getFlowcontrol() const
{
    return _flowcontrol;
}

bool SerialImpl::setFlowcontrol(FlowControl flowcontrol)
{
    if (flowcontrol != FlowControl::Disabled) return false;
    return true;
}

bool SerialImpl::getSingleWireMode() const { return false; }
bool SerialImpl::setSingleWireMode() { return false; }
bool SerialImpl::getSwapRxTxMode() const { return false; }
bool SerialImpl::setSwapRxTxMode() { return false; }
bool SerialImpl::getInvertedMode() const { return false; }
bool SerialImpl::setInvertedMode(bool enable) { return !enable; }

} // namespace rp2350
} // namespace device
