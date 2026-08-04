"""Run the Python port on the same inputs as xcheck.c and assert identical.

Usage: gcc -O2 tests/xcheck.c -o /tmp/xcheck && ./tmp/xcheck | python tests/compare_xcheck.py
(compare_xcheck.py reads the C output on stdin).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_elrs.crc import Crc2Byte
from uav_elrs.fhss import ISM2G4, build_sequence


def main():
    c_lines = sys.stdin.read().strip().splitlines()
    ok = 0
    for line in c_lines:
        parts = line.split()
        if parts[0] == "SEQ":
            seed = int(parts[1])
            c_seq = [int(x) for x in parts[2:]]
            py_seq = build_sequence(seed, ISM2G4)
            assert py_seq == c_seq, f"FHSS mismatch seed {seed}"
            ok += 1
        elif parts[0] == "CRC14":
            c_crc = int(parts[1])
            py = Crc2Byte(14, 0x2E57).calc(
                bytes([0x01, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]), 7, 0x1234 & 0x3FFF)
            assert py == c_crc, f"CRC14 {py} != {c_crc}"
            ok += 1
        elif parts[0] == "CRC16":
            c_crc = int(parts[1])
            py = Crc2Byte(16, 0x3D65).calc(
                bytes([0x01, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]), 11, 0xABCD)
            assert py == c_crc, f"CRC16 {py} != {c_crc}"
            ok += 1
    print(f"XCHECK: {ok} C-vs-Python comparisons all byte-identical")


if __name__ == "__main__":
    main()
