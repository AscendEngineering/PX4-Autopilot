#!/usr/bin/env python3
"""Host tests for px4_uploader.py protocol timing that a real board cannot exercise reliably.

Run: python3 -m unittest -v Tools/test_px4_uploader.py
"""

import importlib.util
import pathlib
import struct
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("px4_uploader", HERE / "px4_uploader.py")
px4_uploader = importlib.util.module_from_spec(spec)
sys.modules["px4_uploader"] = px4_uploader
spec.loader.exec_module(px4_uploader)


class FakeTransport:
    """Answers GET_CRC with a fixed word and INSYNC/OK, recording the timeout of every read."""

    port_name = "fake"
    is_open = True

    def __init__(self, crc: int):
        self.crc = crc
        self.sent = b""
        self.reads = []  # (count, timeout) per recv call
        self._queue = b""

    def send(self, data: bytes) -> None:
        self.sent += data
        if data and data[0] == px4_uploader.BootloaderCommand.GET_CRC:
            self._queue += struct.pack("<I", self.crc) + bytes(
                [px4_uploader.BootloaderResponse.INSYNC, px4_uploader.BootloaderResponse.OK]
            )

    def flush(self) -> None:
        pass

    def recv(self, count: int = 1, timeout=None) -> bytes:
        self.reads.append((count, timeout))
        data, self._queue = self._queue[:count], self._queue[count:]
        return data


class FakeFirmware:
    def __init__(self, crc: int):
        self._crc = crc

    def crc(self, padlen: int) -> int:
        return self._crc


class GetCrcTimeoutTest(unittest.TestCase):
    """The bootloader computes the CRC over the whole flash region before it answers.

    On the RP2350 that is 0.98 s for 3.9 MB; the default 0.5 s sleep plus a 0.5 s
    read leaves about 20 ms of margin. The CRC read must get a timeout that grows
    with the flash size instead of the transport default.
    """

    def _verify(self, fw_maxsize: int) -> FakeTransport:
        transport = FakeTransport(crc=0xA6D1A7BD)
        proto = px4_uploader.BootloaderProtocol(transport, sync_timeout=0.5)
        proto.bl_rev = 5
        proto.fw_maxsize = fw_maxsize
        px4_uploader.time.sleep = lambda s: None  # no real waiting in the test
        proto.verify_crc(FakeFirmware(0xA6D1A7BD))
        return transport

    def test_crc_read_timeout_scales_with_flash_size(self):
        transport = self._verify(3997696)
        crc_reads = [t for count, t in transport.reads if count == 4]
        self.assertEqual(len(crc_reads), 1, "exactly one 4-byte CRC read")
        self.assertIsNotNone(crc_reads[0], "CRC read must not use the transport default timeout")
        self.assertGreaterEqual(crc_reads[0], 4.0, "4 MB of flash needs seconds, not the 0.5 s default")

    def test_crc_read_timeout_never_below_sync_timeout(self):
        transport = self._verify(0)
        crc_reads = [t for count, t in transport.reads if count == 4]
        self.assertGreaterEqual(crc_reads[0], 0.5)

    def test_crc_mismatch_is_still_reported(self):
        transport = FakeTransport(crc=0x12345678)
        proto = px4_uploader.BootloaderProtocol(transport, sync_timeout=0.5)
        proto.bl_rev = 5
        proto.fw_maxsize = 1024
        px4_uploader.time.sleep = lambda s: None
        with self.assertRaises(px4_uploader.ProtocolError):
            proto.verify_crc(FakeFirmware(0xA6D1A7BD))


if __name__ == "__main__":
    unittest.main()
