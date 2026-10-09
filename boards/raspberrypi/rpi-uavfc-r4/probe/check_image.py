#!/usr/bin/env python3
"""Layout checks for the RP2350 probe images (spec 1.5 item 1, phase 2 item 2).

Verifies, on a raw .bin:
  - the five IMAGE_DEF words sit right after the vector table, inside the first 4 KB
  - word 0 (initial MSP) is inside striped SRAM and 8-byte aligned
  - word 1 (reset vector) is odd and inside the image's flash region
  - the image fits its region
"""
import argparse
import struct
import sys

IMAGE_DEF = (0xFFFFDED3, 0x10210142, 0x000001FF, 0x00000000, 0xAB123579)
VECTORS = 16 + 52
ROLES = {
    "bl": (0x10000000, 128 * 1024),
    "app": (0x10020000, 3904 * 1024),
}


def check(path, role):
    origin, length = ROLES[role]
    data = open(path, "rb").read()
    errors = []

    if len(data) > length:
        errors.append(f"image is {len(data)} bytes, region is {length}")

    off = VECTORS * 4
    words = struct.unpack_from("<5I", data, off)
    if words != IMAGE_DEF:
        errors.append(f"IMAGE_DEF at 0x{off:x} is {[hex(w) for w in words]}")
    if off + 20 > 4096:
        errors.append("IMAGE_DEF block ends outside the first 4 KB")

    msp, pc = struct.unpack_from("<2I", data, 0)
    if not (0x20000000 < msp <= 0x2007FFF8) or msp & 7:
        errors.append(f"initial MSP 0x{msp:08x} invalid")
    if not (pc & 1) or not (origin <= (pc & ~1) < origin + length):
        errors.append(f"reset vector 0x{pc:08x} outside region")

    for e in errors:
        print(f"{path}: {e}", file=sys.stderr)
    if not errors:
        print(f"{path}: ok ({len(data)} bytes, msp=0x{msp:08x}, reset=0x{pc:08x})")
    return not errors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=ROLES, required=True)
    ap.add_argument("bin")
    args = ap.parse_args()
    sys.exit(0 if check(args.bin, args.role) else 1)


if __name__ == "__main__":
    main()
