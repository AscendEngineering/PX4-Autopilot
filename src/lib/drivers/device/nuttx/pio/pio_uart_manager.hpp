#pragma once

#include <nuttx/fs/fs.h>
#include <stdio.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the PIO UART drivers and register them in the NuttX VFS.
 * This creates /dev/ttyS2 and /dev/ttyS3.
 */
void pio_uart_manager_init(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
namespace device { namespace rp2350 {

class PioUartManager {
public:
    static void init();
private:
    PioUartManager() = delete;
};

} }
#endif
