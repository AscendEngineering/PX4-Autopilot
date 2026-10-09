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

## Where the fixes live

The same two changes are commits on the local submodule branch `rp2350-backports`
(in `platforms/nuttx/NuttX/nuttx`). To land them: open a pull request against
PX4/NuttX, or push the branch to an Ascend fork and point `.gitmodules` at it on
this branch; then bump the submodule pointer and delete this directory. Until
then CI, which checks out the pristine submodule, fails
`test_nuttx_submodule_patches_applied` by design.
