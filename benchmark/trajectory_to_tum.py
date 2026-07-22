#!/usr/bin/env python3
"""Convert EuRoC ground truth or OpenVINS recorder output to TUM format."""

import argparse
import csv
from pathlib import Path


def convert_euroc(source: Path, destination: Path) -> int:
    count = 0
    with source.open(newline="") as src, destination.open("w") as dst:
        for row in csv.reader(src):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 8:
                continue
            values = [item.strip() for item in row[:8]]
            timestamp = float(values[0]) * 1e-9
            px, py, pz = map(float, values[1:4])
            qw, qx, qy, qz = map(float, values[4:8])
            dst.write(
                f"{timestamp:.9f} {px:.9f} {py:.9f} {pz:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )
            count += 1
    return count


def convert_openvins(source: Path, destination: Path) -> int:
    count = 0
    with source.open() as src, destination.open("w") as dst:
        for line in src:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            fields = line.split()
            if len(fields) < 8:
                continue
            dst.write(" ".join(fields[:8]) + "\n")
            count += 1
    return count


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("format", choices=("euroc", "openvins"))
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()

    args.destination.parent.mkdir(parents=True, exist_ok=True)
    if args.format == "euroc":
        count = convert_euroc(args.source, args.destination)
    else:
        count = convert_openvins(args.source, args.destination)
    if count == 0:
        raise SystemExit(f"no poses converted from {args.source}")
    print(f"converted {count} poses: {args.destination}")


if __name__ == "__main__":
    main()

