#!/usr/bin/env python3
"""
Convert a raw .text binary (as produced by `objcopy -O binary`) into a
.mem file: one 32-bit instruction per line, hex, no '0x' prefix.

Byte order: RISC-V is little-endian, so bytes [b0 b1 b2 b3] in the .bin
file pack into the instruction word b3<<24 | b2<<16 | b1<<8 | b0. That
word is exactly the value the DUT/decoder expect on instr[31:0].

The core's instruction memory holds IMEM_WORDS words (see instruction_mem
in rtl/risc.v); a program that doesn't fit is silently truncated by the
DUT's wrapping write pointer, so this script refuses to emit an
oversized .mem file instead of producing a program that would run wrong.
"""
import struct
import sys

IMEM_WORDS = 256  # must match `parameter DEPTH` in instruction_mem (rtl/risc.v)


def convert(bin_path: str, mem_path: str) -> int:
    with open(bin_path, "rb") as f:
        data = f.read()

    if len(data) % 4 != 0:
        pad = 4 - (len(data) % 4)
        sys.stderr.write(
            f"warning: {bin_path} is {len(data)} bytes, not a multiple of 4; "
            f"padding with {pad} zero byte(s)\n"
        )
        data += b"\x00" * pad

    n_words = len(data) // 4
    if n_words > IMEM_WORDS:
        sys.stderr.write(
            f"error: program has {n_words} instructions, but instruction "
            f"memory only holds {IMEM_WORDS} words (see DEPTH in "
            f"instruction_mem, rtl/risc.v)\n"
        )
        return 1
    if n_words == 0:
        sys.stderr.write(f"error: {bin_path} is empty\n")
        return 1

    words = struct.unpack(f"<{n_words}I", data)
    with open(mem_path, "w") as f:
        for w in words:
            f.write(f"{w:08x}\n")

    print(f"{bin_path}: {n_words} instruction(s) -> {mem_path}")
    return 0


def main() -> int:
    if len(sys.argv) != 3:
        sys.stderr.write(f"usage: {sys.argv[0]} <input.bin> <output.mem>\n")
        return 1
    return convert(sys.argv[1], sys.argv[2])


if __name__ == "__main__":
    sys.exit(main())
