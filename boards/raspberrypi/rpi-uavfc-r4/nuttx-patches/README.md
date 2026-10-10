# NuttX patches the rpi-uavfc-r4 bootloader needs

Backports from upstream apache/nuttx that the PX4/NuttX submodule pin
(platforms/nuttx/NuttX/nuttx) does not have yet:

- `0001`: `exception_direct` declares r0 clobbered. Without it the RP2350
  image panics on its first interrupt.
- `0002`: `usbdev_register` keeps `SIE_CTRL.PULLUP_EN`. Without it the device
  never attaches on USB.
- `0003` to `0005`: the rp23xx USB device driver's bulk endpoint fixes
  (upstream d89019aa78, cfeca506f8, a85b28bc2c). Without them a bulk IN
  transfer can be armed with `AVAILABLE` set in the same write as `FULL`, the
  controller consumes the buffer empty, and every later reply from the
  bootloader is lost: on the bench the uploader's GET_CRC reply arrived and
  the INSYNC after it never did. `0004` also selects
  `ARCH_USBDEV_STALLQUEUE` for `ARCH_CHIP_RP23XX` in `arch/arm/Kconfig`.
- `0006` to `0016`: the USB CDC ACM class driver's fixes (upstream
  dd5a2662ac, e78e152cc7, fe293ec9b5, cc067ab199, d2daf66d38, a53223741e,
  10180ab4eb, 04fbce9d40, ae39301416, d1418d9420, 58d1d9f9d6: every commit
  on `drivers/usbdev/cdcacm.c` after the fork's last one, so the file matches
  upstream master except the `debug.h` include). They close races between
  the transmit completion and the serial layer's send path.
- `0017` and `0018`: NVIC priority initialisation (upstream ed5a661da8,
  614cdcd93f). The fork's loop set the default priority for IRQs 0 to 11
  only; IRQs 12 to 51, the USB controller among them, stayed at priority 0,
  above the BASEPRI level that `up_irq_save()` raises, so no critical
  section ever masked the USB interrupt. On the bench the console's
  transmit side stopped within 20 s of sustained output: the completion
  interrupt ran inside the class driver's locked send path, drained the
  ring, and the thread then queued a zero-length request that the rp23xx
  driver never arms. `0018` also puts PendSV at the lowest priority.

Apply after any `make distclean`, `make submodulesclean` or
`git submodule update --force`:

```
git -C platforms/nuttx/NuttX/nuttx apply ../../../../boards/raspberrypi/rpi-uavfc-r4/nuttx-patches/*.patch
```

`platforms/nuttx/src/bootloader/rpi/rpi_common/tests/test_bootloader.py`
fails with that command when the patches are missing. The permanent fix is
a commit on a NuttX fork and a submodule bump; then this directory goes.

## Where the fixes live

The same changes are commits on the local submodule branch `rp2350-backports`
(in `platforms/nuttx/NuttX/nuttx`); `0003` to `0018` keep their upstream
authorship. To land them: open a pull request against PX4/NuttX, or push the
branch to an Ascend fork and point `.gitmodules` at it on this branch; then
bump the submodule pointer and delete this directory. Until then CI, which
checks out the pristine submodule, fails `test_nuttx_submodule_patches_applied`
by design.
