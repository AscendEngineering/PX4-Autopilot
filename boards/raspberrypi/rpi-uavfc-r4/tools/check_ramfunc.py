#!/usr/bin/env python3
"""Post-link check of the rpi-uavfc-r4 image: SRAM placement and the boot and
XIP-down call graphs. Run by CMake after every link; a violation fails the build.

Usage: check_ramfunc.py ELF MAP [--ld SCRIPT] [--ekf-list LD]

Checks:
  1. the linker boundaries _sramfunc, _eramfunc, _lramfunc exist and are ordered;
  2. every placement rule between them in the linker script matched at least
     one input section, and none of its matches ended up in flash;
  3. every symbol in the generated EKF list sits inside [_sramfunc, _eramfunc);
  4. boot path (flash only): nothing executed before copy_ram_code() returns
     may be in SRAM. Inside __start the boundary is positional: call sites
     before the bl to rp23xx_boardearlyinitialize are before the copy. The
     other roots are walked fully. Linker veneers are resolved to their final
     target; any transfer with an unknown target fails;
  5. XIP-down path (SRAM only): flash_op and the park handler, everything they
     reach, and every literal they load must be in SRAM or ROM. Indirect calls
     are allowed only in flash_op, whose contract (rpi_flash.h) is resolved
     boot-ROM entry points and the SRAM stack copy of the XIP setup routine.
"""
import argparse
import fnmatch
import re
import struct
import subprocess
import sys

from symbols import (archive_for, archive_ranges_from_map, classify, load_absolute_symbols, load_symtab,
                     map_input_sections, symbolize)

BOUNDARIES = ("_sramfunc", "_eramfunc", "_lramfunc")
BOUNDARY_CALL = "rp23xx_boardearlyinitialize"
COPY_FN = "copy_ram_code"
FLASH_ONLY_ROOTS = ("rp23xx_clockconfig", "rp23xx_boardearlyinitialize", "copy_ram_code")
SRAM_ONLY_ROOTS = ("flash_op",)
SRAM_ONLY_OPTIONAL = ("rpi_flash_park_handler",)
INDIRECT_OK = {"flash_op"}     # documented contract: ROM entries and the XIP setup stack copy
# An assertion before the copy ends in a reset either way; NuttX's panic path runs
# notifier chains through function pointers, so the boot walk stops at these.
TERMINAL = {"__assert", "_assert", "abort", "_exit", "exit", "up_assert", "panic"}

_INSN = re.compile(r"^\s*([0-9a-f]+):\s+(\S+)\s*(.*)$")
_FUNC = re.compile(r"^([0-9a-f]+) <([^>]+)>:$")
_TARGET = re.compile(r"^([0-9a-f]+)\s+<")
_LDR_PC_LIT = re.compile(r"^\[pc(?:,\s*#(-?\d+))?\]")
_WORD = re.compile(r"^\s*([0-9a-f]+):\s+\.word\s+0x([0-9a-f]+)")
RETURN_INSNS = ("bx\tlr", "bx lr")


class Elf:
    """Just enough ELF32 little-endian reading to fetch words by virtual address."""

    def __init__(self, path):
        self.data = open(path, "rb").read()
        d = self.data
        assert d[:4] == b"\x7fELF" and d[4] == 1 and d[5] == 1, "ELF32 little-endian expected"
        shoff, shentsize, shnum = struct.unpack_from("<I", d, 0x20)[0], struct.unpack_from("<H", d, 0x2E)[0], struct.unpack_from("<H", d, 0x30)[0]
        self.sections = []
        for i in range(shnum):
            off = shoff + i * shentsize
            _, sh_type, sh_flags, sh_addr, sh_offset, sh_size = struct.unpack_from("<IIIIII", d, off)
            if sh_type == 1 and sh_flags & 2 and sh_size:          # PROGBITS, ALLOC
                self.sections.append((sh_addr, sh_size, sh_offset))

    def word(self, addr: int):
        for base, size, off in self.sections:
            if base <= addr < base + size - 3:
                return struct.unpack_from("<I", self.data, off + addr - base)[0]
        return None


class Function:
    __slots__ = ("name", "start", "size", "insns")

    def __init__(self, name, start, size):
        self.name, self.start, self.size, self.insns = name, start, size, []

    def contains(self, addr):
        return self.start <= addr < self.start + self.size


def disassemble(elf: str) -> dict:
    """{start: Function} from objdump, each with (addr, mnemonic, operands) tuples."""
    text = subprocess.run(["arm-none-eabi-objdump", "-d", "--no-show-raw-insn", elf],
                          check=True, capture_output=True, text=True).stdout
    funcs, cur = {}, None
    for line in text.splitlines():
        m = _FUNC.match(line)
        if m:
            cur = Function(m.group(2), int(m.group(1), 16), 0)
            funcs[cur.start] = cur
            continue
        if cur is None:
            continue
        m = _INSN.match(line)
        if m:
            cur.insns.append((int(m.group(1), 16), m.group(2), m.group(3)))
    return funcs


def _size_functions(funcs: dict, symtab):
    sized = {s[0]: (s[1], s[2]) for s in symtab}
    for start, f in funcs.items():
        if start in sized:
            f.size = sized[start][0]
        elif f.insns:
            last = f.insns[-1][0]
            f.size = last + 4 - start
    return funcs


def veneer_target(f: Function, elf: Elf):
    """Final target of a linker long-branch veneer (ldr.w pc, [pc, #imm] + literal), or None."""
    for addr, mn, ops in f.insns:
        if mn.startswith("ldr") and ops.startswith("pc,"):
            rest = ops[3:].strip()
            m = _LDR_PC_LIT.match(rest)
            if m:
                imm = int(m.group(1) or 0)
                lit = ((addr & ~3) + 4 + imm)
                w = elf.word(lit)
                return None if w is None else w & ~1
    return None


def transfers(f: Function, elf: Elf):
    """Yield (site, kind, target_or_None, text) for every control transfer that may leave f."""
    for addr, mn, ops in f.insns:
        base = mn.split(".")[0]
        if base in ("bl", "blx") and _TARGET.match(ops):
            yield addr, "call", int(_TARGET.match(ops).group(1), 16), f"{mn} {ops}"
        elif base == "blx":
            yield addr, "indirect", None, f"{mn} {ops}"
        elif base == "bx":
            if ops.strip() != "lr":
                yield addr, "indirect", None, f"{mn} {ops}"
        elif base.startswith("b") and base not in ("bl", "blx", "bx", "bkpt") and _TARGET.match(ops):
            tgt = int(_TARGET.match(ops).group(1), 16)
            if not f.contains(tgt):
                yield addr, "branch", tgt, f"{mn} {ops}"
        elif base in ("cbz", "cbnz", "tbb", "tbh"):
            continue                                    # in-function only
        elif base == "pop" and "pc" in ops:
            continue                                    # return
        elif base.startswith("ldr") and ops.startswith("pc,"):
            m = _LDR_PC_LIT.match(ops[3:].strip())
            w = elf.word((addr & ~3) + 4 + int(m.group(1) or 0)) if m else None
            if w is not None:
                # a literal jump: a veneer body or a trampoline; resolve it like a branch
                yield addr, "branch", w & ~1, f"{mn} {ops}"
            else:
                yield addr, "indirect", None, f"{mn} {ops}"
        elif base == "mov" and ops.startswith("pc,"):
            yield addr, "indirect", None, f"{mn} {ops}"


def literal_loads(f: Function, elf: Elf):
    """(site, literal address, word) for every pc-relative load in f."""
    for addr, mn, ops in f.insns:
        if not mn.startswith("ldr"):
            continue
        parts = ops.split(",", 1)
        if len(parts) < 2:
            continue
        m = _LDR_PC_LIT.match(parts[1].strip())
        if m:
            lit = (addr & ~3) + 4 + int(m.group(1) or 0)
            yield addr, lit, elf.word(lit)


def resolve(target: int, funcs: dict, elf: Elf, symtab):
    """Follow veneers; return (final_addr, name) for a direct target address."""
    seen = 0
    while seen < 4:
        f = funcs.get(target & ~1)
        if f is None:
            s = symbolize(target, symtab)
            return target & ~1, (s[0] if s else None)
        if f.name.endswith("_veneer"):
            t = veneer_target(f, elf)
            if t is None:
                return target & ~1, f.name
            target = t
            seen += 1
            continue
        return f.start, f.name
    return target & ~1, None


def walk(root: Function, funcs: dict, elf: Elf, symtab, phase: str, allowed, violations, path=None,
         boundary=None, visited=None):
    """Depth-first over calls and tail calls. `allowed(addr) -> bool` decides a target's region.
    `boundary` is a dict {"name": fn, "found": bool}: in the function that calls `name`,
    transfers at or after that call site are not checked (the positional copy boundary),
    and the walk does not descend into it. Targets in TERMINAL are not descended into."""
    visited = visited if visited is not None else set()
    path = (path or []) + [root.name]
    if root.start in visited:
        return
    visited.add(root.start)
    boundary_site = None
    if boundary is not None:
        for site, kind, tgt, _ in transfers(root, elf):
            if tgt is not None and resolve(tgt, funcs, elf, symtab)[1] == boundary["name"]:
                boundary_site = site
                boundary["found"] = True
                break
    for site, kind, tgt, text in transfers(root, elf):
        if boundary_site is not None and site >= boundary_site:
            break
        where = f"{phase}: {' > '.join(path)} @0x{site:08x}"
        if kind == "indirect":
            if root.name in INDIRECT_OK and phase == "xip_down":
                continue
            violations.append(f"{where}: unknown indirect transfer `{text}`")
            continue
        final, name = resolve(tgt, funcs, elf, symtab)
        label = name or "?"
        if not allowed(final):
            violations.append(f"{where}: -> {label} (0x{final:08x}) is in {classify(final).upper()} `{text}`")
            continue
        if name in TERMINAL:
            continue
        callee = funcs.get(final)
        if callee is not None:
            walk(callee, funcs, elf, symtab, phase, allowed, violations, path, boundary, visited)


# ---------------------------------------------------------------- rules and lists

def parse_rules(ld_text: str) -> list:
    """Placement rules between _sramfunc and _eramfunc: lines starting with `*`."""
    rules, inside = [], False
    for line in ld_text.splitlines():
        t = line.strip()
        if "_sramfunc" in t and "ABSOLUTE" in t:
            inside = True
            continue
        if "_eramfunc" in t and "ABSOLUTE" in t:
            break
        if inside and t.startswith("*") and not re.match(r"^\*(\s|/|$)", t):
            rules.append(t)     # a placement rule, not a comment continuation line
    return rules


_RULE = re.compile(r"^(?P<file>[^:(]*)(?::(?P<obj>[^(]*))?\((?P<secs>[^)]*)\)$")


def rule_matches(rule: str, sec) -> bool:
    m = _RULE.match(rule.strip())
    if not m:
        return False
    fpat = m.group("file") or "*"
    opat = m.group("obj")
    secs = m.group("secs").split()
    if not any(fnmatch.fnmatchcase(sec.name, sp) for sp in secs):
        return False
    if fpat != "*" and not fnmatch.fnmatchcase(sec.archive, fpat) and not fnmatch.fnmatchcase(sec.path, fpat):
        return False
    if opat and not fnmatch.fnmatchcase(sec.obj, opat):
        return False
    return True


def placement_violations(rules: list, sections: list, lo: int, hi: int) -> list:
    out = []
    live = [s for s in sections if s.addr != 0]     # address 0: garbage-collected, listed as discarded
    for rule in rules:
        matched = [s for s in live if rule_matches(rule, s)]
        if not matched:
            out.append(f"rule matched nothing: {rule}")
            continue
        for s in matched:
            if not lo <= s.addr < hi:
                out.append(f"rule matched text still in flash: {rule} -> {s.name} at 0x{s.addr:08x}")
    return out


def ekf_list_violations(ld_text: str, symtab, lo: int, hi: int) -> list:
    out = []
    by_name = {s[2]: s for s in symtab}
    for line in ld_text.splitlines():
        m = re.match(r"^\s*\*\(\.text\.(\S+)\)", line)
        if not m:
            continue
        name = m.group(1)
        s = by_name.get(name)
        if s is None:
            out.append(f"{name}: not in the ELF")
        elif not lo <= s[0] < hi:
            out.append(f"{name} placed at 0x{s[0]:08x}, outside .ramfunc")
    return out


# ---------------------------------------------------------------- top level

def check(elf_path: str, map_path: str, ld_rules: list, ekf_list_text: str = "",
          required_symbols=BOUNDARIES) -> list:
    violations = []
    nm = subprocess.run(["arm-none-eabi-nm", "-S", "--defined-only", elf_path],
                        check=True, capture_output=True, text=True).stdout
    symtab = load_symtab(nm)
    absolute = load_absolute_symbols(nm)
    for name in required_symbols:
        if name not in absolute:
            violations.append(f"boundary {name} missing from the ELF")
    if violations:
        return violations
    lo, hi, lma = (absolute[n] for n in BOUNDARIES)
    if not (classify(lo) == "sram" and classify(hi) == "sram" and lo <= hi):
        violations.append(f"boundaries out of order or outside SRAM: _sramfunc 0x{lo:08x} _eramfunc 0x{hi:08x}")
    if classify(lma) != "flash":
        violations.append(f"_lramfunc 0x{lma:08x} is not in flash")

    with open(map_path) as f:
        sections = map_input_sections(f.read())
    violations += placement_violations(ld_rules, sections, lo, hi)
    if ekf_list_text:
        violations += ekf_list_violations(ekf_list_text, symtab, lo, hi)

    elf = Elf(elf_path)
    funcs = _size_functions(disassemble(elf_path), symtab)
    by_name = {f.name: f for f in funcs.values()}

    # 4. boot path, flash only
    in_flash = lambda a: classify(a) == "flash"
    reset = elf.word(absolute.get("_stext", 0x10020000) + 4)
    start = funcs.get((reset or 0) & ~1)
    if start is None:
        violations.append("reset handler from the vector table not found in the disassembly")
    else:
        if not in_flash(start.start):
            violations.append(f"before_copy: reset handler {start.name} is in SRAM (0x{start.start:08x})")
        boundary = {"name": BOUNDARY_CALL, "found": False}
        walk(start, funcs, elf, symtab, "before_copy", in_flash, violations, boundary=boundary)
        if not boundary["found"]:
            violations.append(f"before_copy: no call to the copy boundary {BOUNDARY_CALL} reachable from the reset handler {start.name}")
    for name in FLASH_ONLY_ROOTS:
        f = by_name.get(name)
        if f is None:
            violations.append(f"before_copy: {name} missing from the ELF (inlined or renamed)")
            continue
        if not in_flash(f.start):
            violations.append(f"before_copy: {name} is in SRAM (0x{f.start:08x})")
        walk(f, funcs, elf, symtab, "before_copy", in_flash, violations, visited=set())
    copy_fn, early = by_name.get(COPY_FN), by_name.get(BOUNDARY_CALL)
    if copy_fn is not None and early is not None:
        lits = {w for fn in (copy_fn, early) for _, _, w in literal_loads(fn, elf) if w is not None}
        for n in BOUNDARIES:
            if absolute[n] not in lits:
                violations.append(f"before_copy: {COPY_FN} range does not cover {n} (0x{absolute[n]:08x} not loaded by {BOUNDARY_CALL} or {COPY_FN})")

    # 5. XIP-down path, SRAM or ROM only
    sram_or_rom = lambda a: classify(a) in ("sram", "rom")
    roots = list(SRAM_ONLY_ROOTS) + [n for n in SRAM_ONLY_OPTIONAL if n in by_name]
    for name in roots:
        f = by_name.get(name)
        if f is None:
            violations.append(f"xip_down: {name} missing from the ELF")
            continue
        if not lo <= f.start < hi:
            violations.append(f"xip_down: {name} is not in .ramfunc (0x{f.start:08x})")
        visited = set()
        walk(f, funcs, elf, symtab, "xip_down", sram_or_rom, violations, visited=visited)
        for fstart in visited:
            fn = funcs[fstart]
            for site, lit, w in literal_loads(fn, elf):
                if classify(lit) != "sram":
                    violations.append(f"xip_down: {fn.name} @0x{site:08x}: literal pool at 0x{lit:08x} is not in SRAM")
                elif w is not None and classify(w & ~1) == "flash":
                    violations.append(f"xip_down: {fn.name} @0x{site:08x}: flash data dependency, literal 0x{w:08x}")
    return violations


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("elf")
    ap.add_argument("map")
    ap.add_argument("--ld", help="board linker script to read placement rules from")
    ap.add_argument("--ekf-list", help="generated EKF list to verify by address")
    args = ap.parse_args(argv)
    rules = parse_rules(open(args.ld).read()) if args.ld else []
    ekf = open(args.ekf_list).read() if args.ekf_list else ""
    v = check(args.elf, args.map, rules, ekf)
    if v:
        print(f"check_ramfunc: {len(v)} violation(s):", file=sys.stderr)
        for line in v:
            print(f"  {line}", file=sys.stderr)
        return 1
    print(f"check_ramfunc: ok ({len(rules)} placement rules, {args.ekf_list or 'no EKF list'})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
