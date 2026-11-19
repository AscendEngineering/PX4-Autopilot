#!/usr/bin/env python3
"""Symbol and link-map helpers for check_ramfunc.py.

Names are kept mangled throughout.
"""
import bisect
import os
import re

FLASH = (0x10000000, 0x11000000)
SRAM = (0x20000000, 0x20082000)   # SRAM0-7 striped plus SRAM8 and SRAM9
ROM = (0x00000000, 0x00008000)

_TEXT_TYPES = set("tTwW")
_MAP_CONT = re.compile(r"^\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S+)")
_MAP_ANY_TEXT_ONE_LINE = re.compile(r"^ (\.text(?:\.\S+)?)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S+)")
_MAP_ANY_TEXT_ONLY = re.compile(r"^ (\.text(?:\.\S+)?)\s*$")
_MAP_SEC_ONE_LINE = re.compile(r"^ (\.[A-Za-z_][\w.$]*)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S+)")
_MAP_SEC_ONLY = re.compile(r"^ (\.[A-Za-z_][\w.$]*)\s*$")


def classify(addr: int) -> str:
    if FLASH[0] <= addr < FLASH[1]:
        return "flash"
    if SRAM[0] <= addr < SRAM[1]:
        return "sram"
    if ROM[0] <= addr < ROM[1]:
        return "rom"
    return "invalid"


def load_symtab(nm_text: str) -> list:
    """Sized text symbols from ``nm -S --defined-only`` as (start, size, name), sorted."""
    out = []
    for line in nm_text.splitlines():
        parts = line.split(None, 3)
        if len(parts) < 4:
            continue
        addr, size, typ, name = parts
        if typ not in _TEXT_TYPES:
            continue
        try:
            out.append((int(addr, 16) & ~1, int(size, 16), name.strip()))
        except ValueError:
            continue
    out.sort()
    return out


def load_absolute_symbols(nm_text: str) -> dict:
    """``A``/``a`` symbols, which nm prints without a size column."""
    out = {}
    for line in nm_text.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1] in ("A", "a"):
            out[parts[2]] = int(parts[0], 16)
        elif len(parts) == 4 and parts[2] in ("A", "a"):
            out[parts[3]] = int(parts[0], 16)
    return out


def symbolize(addr: int, symtab: list):
    """(name, start, size) of the sized text symbol containing ``addr``, else None."""
    addr &= ~1
    starts = [s[0] for s in symtab]
    i = bisect.bisect_right(starts, addr) - 1
    if i < 0:
        return None
    start, size, name = symtab[i]
    if start <= addr < start + size:
        return (name, start, size)
    return None


def archive_ranges_from_map(map_text: str) -> list:
    """(start, size, archive) for every ``.text*`` input section, sorted by start.

    Covers objects built without -ffunction-sections (libgcc, libm, assembly),
    whose functions share one plain ``.text`` and never appear by name."""
    out = []
    pending = False
    for line in map_text.splitlines():
        m = _MAP_ANY_TEXT_ONE_LINE.match(line)
        if m:
            start, size = int(m.group(2), 16), int(m.group(3), 16)
            if size:
                out.append((start, size, _archive_basename(m.group(4))))
            pending = False
            continue
        if _MAP_ANY_TEXT_ONLY.match(line):
            pending = True
            continue
        if pending:
            m = _MAP_CONT.match(line)
            if m:
                start, size = int(m.group(1), 16), int(m.group(2), 16)
                if size:
                    out.append((start, size, _archive_basename(m.group(3))))
            pending = False
    out.sort()
    return out


def archive_for(addr: int, ranges: list):
    starts = [r[0] for r in ranges]
    i = bisect.bisect_right(starts, addr) - 1
    if i >= 0 and ranges[i][0] <= addr < ranges[i][0] + ranges[i][1]:
        return ranges[i][2]
    return None


class InputSection:
    """One input section line of a GNU ld map: name, address, size, file and object."""
    __slots__ = ("name", "addr", "size", "path", "archive", "obj")

    def __init__(self, name, addr, size, path):
        self.name, self.addr, self.size, self.path = name, addr, size, path
        base, _, obj = path.partition("(")
        self.archive = os.path.basename(base)
        self.obj = obj.rstrip(")") if obj else ""

    def __repr__(self):
        return f"InputSection({self.name}, 0x{self.addr:08x}, 0x{self.size:x}, {self.path})"


def map_input_sections(map_text: str) -> list:
    """Every input section line of the map, one-line or two-line form, size > 0."""
    out = []
    pending = None
    for line in map_text.splitlines():
        m = _MAP_SEC_ONE_LINE.match(line)
        if m:
            size = int(m.group(3), 16)
            if size:
                out.append(InputSection(m.group(1), int(m.group(2), 16), size, m.group(4)))
            pending = None
            continue
        m = _MAP_SEC_ONLY.match(line)
        if m:
            pending = m.group(1)
            continue
        if pending is not None:
            m = _MAP_CONT.match(line)
            if m:
                size = int(m.group(2), 16)
                if size:
                    out.append(InputSection(pending, int(m.group(1), 16), size, m.group(3)))
            pending = None
    return out


def _archive_basename(path: str) -> str:
    return os.path.basename(path.split("(")[0])


