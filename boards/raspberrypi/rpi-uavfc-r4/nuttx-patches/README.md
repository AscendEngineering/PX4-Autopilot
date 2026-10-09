# NuttX patches the rpi-uavfc-r4 bootloader needs

Two one-line backports from upstream apache/nuttx that the PX4/NuttX
submodule pin (platforms/nuttx/NuttX/nuttx) does not have yet. Without them
the RP2350 image panics on its first interrupt and never attaches on USB.

Apply after any `make distclean`, `make submodulesclean` or
`git submodule update --force`:

```
git -C platforms/nuttx/NuttX/nuttx apply ../../../../boards/raspberrypi/rpi-uavfc-r4/nuttx-patches/*.patch
```

`platforms/nuttx/src/bootloader/rpi/rpi_common/tests/test_bootloader.py`
fails with that command when the patches are missing. The permanent fix is
a commit on a NuttX fork and a submodule bump; then this directory goes.
