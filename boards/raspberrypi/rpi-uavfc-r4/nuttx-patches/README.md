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
(in `platforms/nuttx/NuttX/nuttx`); `0003` to `0005` keep their upstream
authorship. To land them: open a pull request against PX4/NuttX, or push the
branch to an Ascend fork and point `.gitmodules` at it on this branch; then
bump the submodule pointer and delete this directory. Until then CI, which
checks out the pristine submodule, fails `test_nuttx_submodule_patches_applied`
by design.
