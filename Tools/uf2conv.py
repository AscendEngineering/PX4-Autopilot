#!/usr/bin/env python3
############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

"""
Dependency-free UF2 converter for RP2350 / RP2040 flash images.

    uf2conv.py image.bin --base 0x10000000 -o image.uf2
    uf2conv.py --decode image.uf2 -o image.bin
    uf2conv.py --self-test

Follows the RP2350 datasheet section 5.5.2: family ID present, 256-byte
payload per 512-byte block, blocks targeted at 256-byte alignments. The
input is padded to a multiple of 256 bytes with 0xff (erased flash).
"""

import argparse
import os
import struct
import sys

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_NOT_MAIN_FLASH = 0x00000001
UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000

BLOCK_SIZE = 512
PAYLOAD_SIZE = 256
HEADER = "<8I"           # magic0, magic1, flags, target, payload, block_no, num_blocks, family
HEADER_SIZE = struct.calcsize(HEADER)
DATA_FIELD = BLOCK_SIZE - HEADER_SIZE - 4   # 476

FAMILIES = {
    "rp2040": 0xE48BFF56,
    "rp2350-arm-s": 0xE48BFF59,
    "rp2350-riscv": 0xE48BFF5A,
    "rp2350-arm-ns": 0xE48BFF5B,
}


def encode(data, base, family_id):
    if base % PAYLOAD_SIZE:
        raise ValueError(f"base 0x{base:x} is not 256-byte aligned")
    pad = (-len(data)) % PAYLOAD_SIZE
    data = bytes(data) + b"\xff" * pad
    num_blocks = len(data) // PAYLOAD_SIZE
    out = bytearray()
    for i in range(num_blocks):
        chunk = data[i * PAYLOAD_SIZE:(i + 1) * PAYLOAD_SIZE]
        out += struct.pack(HEADER, UF2_MAGIC_START0, UF2_MAGIC_START1,
                           UF2_FLAG_FAMILY_ID_PRESENT, base + i * PAYLOAD_SIZE,
                           PAYLOAD_SIZE, i, num_blocks, family_id)
        out += chunk + b"\x00" * (DATA_FIELD - PAYLOAD_SIZE)
        out += struct.pack("<I", UF2_MAGIC_END)
    return bytes(out)


def decode(blob):
    """Decode a contiguous flash image with only the family-ID flag set.

    Returns (base, data, family_id). Rejects malformed or unsupported files,
    including NOT_MAIN_FLASH blocks, which must not become firmware bytes.
    Identical retransmissions and out-of-order blocks are supported.
    """
    if not blob or len(blob) % BLOCK_SIZE:
        raise ValueError("file size must be a non-zero multiple of 512")
    blocks = {}
    block_numbers = {}
    family = None
    num_blocks = None
    for off in range(0, len(blob), BLOCK_SIZE):
        b = blob[off:off + BLOCK_SIZE]
        m0, m1, flags, target, payload, no, total, fam = struct.unpack_from(HEADER, b, 0)
        (mend,) = struct.unpack_from("<I", b, BLOCK_SIZE - 4)
        if (m0, m1, mend) != (UF2_MAGIC_START0, UF2_MAGIC_START1, UF2_MAGIC_END):
            raise ValueError(f"bad magic in block at offset {off}")
        if not flags & UF2_FLAG_FAMILY_ID_PRESENT:
            raise ValueError(f"block {no} has no family ID")
        if flags & UF2_FLAG_NOT_MAIN_FLASH:
            raise ValueError(f"block {no} is not firmware (NOT_MAIN_FLASH)")
        if flags != UF2_FLAG_FAMILY_ID_PRESENT:
            raise ValueError(f"block {no} has unsupported flags 0x{flags:x}")
        if payload != PAYLOAD_SIZE:
            raise ValueError(f"block {no} payload is {payload}, expected 256")
        if total == 0 or no >= total:
            raise ValueError(f"block number {no} outside declared count {total}")
        if target % PAYLOAD_SIZE or not (0x10000000 <= target < 0x12000000):
            raise ValueError(f"block {no} has invalid flash target 0x{target:08x}")
        if family is None:
            family, num_blocks = fam, total
        elif (fam, total) != (family, num_blocks):
            raise ValueError(f"block {no} changes family or block count")
        content = b[HEADER_SIZE:HEADER_SIZE + PAYLOAD_SIZE]
        if no in block_numbers:
            if block_numbers[no] != (target, content):
                raise ValueError(f"conflicting retransmission of block {no}")
            continue
        if target in blocks:
            raise ValueError(f"multiple block numbers target 0x{target:08x}")
        block_numbers[no] = (target, content)
        blocks[target] = content
    if num_blocks != len(block_numbers):
        raise ValueError(f"{len(block_numbers)} block numbers present, header says {num_blocks}")
    base = min(blocks)
    data = bytearray()
    for i in range(num_blocks):
        addr = base + i * PAYLOAD_SIZE
        if addr not in blocks:
            raise ValueError(f"missing block for 0x{addr:08x}")
        data += blocks[addr]
    return base, bytes(data), family


def self_test():
    import random
    rng = random.Random(1)
    for length in (0, 1, 255, 256, 257, 4096, 12345):
        data = bytes(rng.getrandbits(8) for _ in range(length))
        base = 0x10020000
        fam = FAMILIES["rp2350-arm-s"]
        blob = encode(data, base, fam)
        expected_blocks = (length + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
        assert len(blob) == expected_blocks * BLOCK_SIZE, length
        if expected_blocks == 0:
            continue
        rbase, rdata, rfam = decode(blob)
        assert rbase == base and rfam == fam, length
        assert rdata[:length] == data, length
        assert rdata[length:] == b"\xff" * (len(rdata) - length), length
        for i in range(expected_blocks):
            target = struct.unpack_from("<I", blob, i * BLOCK_SIZE + 12)[0]
            assert target == base + i * PAYLOAD_SIZE, (length, i)
    try:
        encode(b"x", 0x10000010, fam)
    except ValueError:
        pass
    else:
        raise AssertionError("unaligned base accepted")

    # Exercise the ROM transfer rules, not just encode/decode agreement.
    data = bytes(rng.getrandbits(8) for _ in range(2 * PAYLOAD_SIZE))
    blob = encode(data, base, fam)
    expected = (base, data, fam)
    assert decode(blob[BLOCK_SIZE:] + blob[:BLOCK_SIZE]) == expected
    assert decode(blob + blob[:BLOCK_SIZE]) == expected  # identical retransmission

    def changed_word(offset, value):
        changed = bytearray(blob)
        struct.pack_into("<I", changed, offset, value)
        return changed

    invalid = {
        "empty": b"",
        "truncated": blob[:-1],
        "missing block": blob[:BLOCK_SIZE],
        "duplicate number, different target": changed_word(BLOCK_SIZE + 20, 0),
        "number out of range": changed_word(20, 2),
        "zero count": changed_word(24, 0),
        "changed count": changed_word(BLOCK_SIZE + 24, 3),
        "changed family": changed_word(BLOCK_SIZE + 28, FAMILIES["rp2040"]),
        "missing family flag": changed_word(8, 0),
        "not main flash": changed_word(8, UF2_FLAG_FAMILY_ID_PRESENT | UF2_FLAG_NOT_MAIN_FLASH),
        "unsupported flags": changed_word(8, UF2_FLAG_FAMILY_ID_PRESENT | 0x1000),
        "unaligned target": changed_word(12, base + 1),
        "non-flash target": changed_word(12, 0x20000000),
        "duplicate target": changed_word(BLOCK_SIZE + 12, base),
        "address gap": changed_word(BLOCK_SIZE + 12, base + 2 * PAYLOAD_SIZE),
        "bad magic": changed_word(0, 0),
        "bad payload size": changed_word(16, PAYLOAD_SIZE - 1),
        "conflicting retransmission": blob + changed_word(HEADER_SIZE, 0)[:BLOCK_SIZE],
    }
    for name, malformed in invalid.items():
        try:
            decode(malformed)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted {name}")
    print("uf2conv self-test ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?", help=".bin to encode, or .uf2 with --decode")
    ap.add_argument("-o", "--output")
    ap.add_argument("--base", type=lambda s: int(s, 0), default=0x10000000, help="load address (default 0x10000000)")
    ap.add_argument("--family", choices=sorted(FAMILIES), default="rp2350-arm-s")
    ap.add_argument("--decode", action="store_true", help="convert a .uf2 back to a .bin")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        self_test()
        return
    if not args.input:
        ap.error("input is required")

    blob = open(args.input, "rb").read()
    if args.decode:
        base, data, fam = decode(blob)
        name = next((k for k, v in FAMILIES.items() if v == fam), hex(fam))
        out = args.output or os.path.splitext(args.input)[0] + ".bin"
        open(out, "wb").write(data)
        print(f"{out}: base 0x{base:08x}, {len(data)} bytes, family {name}")
    else:
        out = args.output or os.path.splitext(args.input)[0] + ".uf2"
        uf2 = encode(blob, args.base, FAMILIES[args.family])
        open(out, "wb").write(uf2)
        print(f"{out}: base 0x{args.base:08x}, {len(uf2) // BLOCK_SIZE} blocks, family {args.family}")


if __name__ == "__main__":
    main()
