#!/usr/bin/env python3
# tools/check_stl.py
"""Print the bounding-box extents of a binary STL. Usage: check_stl.py file.stl"""
import struct, sys


def bbox(path):
    with open(path, "rb") as f:
        f.read(80)
        (n,) = struct.unpack("<I", f.read(4))
        mins, maxs = [1e30] * 3, [-1e30] * 3
        for _ in range(n):
            rec = struct.unpack("<12fH", f.read(50))
            for v in range(3):
                for k in range(3):
                    val = rec[3 + v * 3 + k]
                    mins[k] = min(mins[k], val)
                    maxs[k] = max(maxs[k], val)
    return [maxs[k] - mins[k] for k in range(3)]


if __name__ == "__main__":
    print(["%.2f" % e for e in bbox(sys.argv[1])])
