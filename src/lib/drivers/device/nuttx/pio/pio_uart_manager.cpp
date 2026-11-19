#include "pio_uart_manager.hpp"
#include "pio_serial.h"
#include <nuttx/fs/fs.h>
#include <stdio.h>
#include <errno.h>

using namespace device::rp2350;

static PIOUartDriver* g_pio_uart_instances[PIO_NUM_INSTANCES] = {nullptr};

// --- VFS SHIM FUNCTIONS ---
// Adjusted signatures to match the expected NuttX file_operations
// Based on the error: ssize_t (*)(file*, char*, size_t)

static ssize_t pio_uart_read(struct file *file, char *buf, size_t size) {
    uint8_t instance = (uint8_t)(uintptr_t)file->f_inode->i_private;
    if (instance >= PIO_NUM_INSTANCES) return -EINVAL;

    PIOUartDriver* driver = g_pio_uart_instances[instance];
    if (!driver) return -ENODEV;

    ssize_t bytes_read = driver->read_bytes((uint8_t*)buf, (uint16_t)size);
    if (bytes_read < 0) return -EIO;

    return bytes_read;
}

static ssize_t pio_uart_write(struct file *file, const char *buf, size_t size) {
    uint8_t instance = (uint8_t)(uintptr_t)file->f_inode->i_private;
    if (instance >= PIO_NUM_INSTANCES) return -EINVAL;

    PIOUartDriver* driver = g_pio_uart_instances[instance];
    if (!driver) return -ENODEV;

    return driver->write_bytes(buf, size);
}

static int pio_uart_ioctl(struct file *file, int cmd, unsigned long arg) {
    return -ENOTTY;
}

static struct file_operations pio_uart_fops = {
    .read = pio_uart_read,
    .write = pio_uart_write,
    .ioctl = pio_uart_ioctl,
};

// --- MANAGER IMPLEMENTATION ---

void PioUartManager::init() {
    for (uint8_t i = 0; i < 2; i++) {
        char devname[16];
        snprintf(devname, sizeof(devname), "/dev/ttyS%d", i + 2);

        g_pio_uart_instances[i] = new PIOUartDriver(devname, 57600, i);

        if (!g_pio_uart_instances[i]) {
            printf("PIO UART: Failed to allocate driver for %s\n", devname);
            continue;
        }

        if (register_driver(devname, &pio_uart_fops, 0666, (void*)(uintptr_t)i) < 0) {
            printf("PIO UART: Failed to register driver for %s\n", devname);
            delete g_pio_uart_instances[i];
            g_pio_uart_instances[i] = nullptr;
        } else {
            printf("PIO UART: Registered %s (instance %u)\n", devname, i);
        }
    }
}

// C wrapper for init.c
void pio_uart_manager_init(void) {
    PioUartManager::init();
}
