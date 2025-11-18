#!/usr/bin/env python3
from pathlib import Path

KEEP_COLS = 8
HEADER_DEFAULT = ["timestamp(s)", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]

INPUT_PATH_02 = "/home/class_ee/workspace/results/exp02_construction_multilevel_estimate.txt"
OUTPUT_PATH_02 = "/home/class_ee/workspace/results/filtered/exp02_construction_multilevel.txt"
INPUT_PATH_15 = "/home/class_ee/workspace/results/exp15_attic_to_upper_gallery_estimate.txt"
OUTPUT_PATH_15 = "/home/class_ee/workspace/results/filtered/exp15_attic_to_upper_gallery.txt"
INPUT_PATH_21 = "/home/class_ee/workspace/results/exp21_outside_building_estimate.txt"
OUTPUT_PATH_21 = "/home/class_ee/workspace/results/filtered/exp21_outside_building.txt"

def process_file(src_path: str, dst_path: str):
    """Read src, keep only first 8 columns, write to dst with a # header."""
    src = Path(src_path)
    dst = Path(dst_path)

    if not src.is_file():
        raise FileNotFoundError(f"Input file not found: {src}")

    dst.parent.mkdir(parents=True, exist_ok=True)

    wrote_header = False

    with src.open("r", encoding="utf-8", errors="ignore") as fin, \
         dst.open("w", encoding="utf-8") as fout:

        for line in fin:
            s = line.strip()
            if not s:
                continue

            # Header or comment line starting with '#'
            if s.startswith("#"):
                tokens = s.lstrip("#").strip().split()
                if tokens:
                    header = tokens[:KEEP_COLS]
                    if len(header) < KEEP_COLS:
                        # Pad with default header names if needed
                        for i in range(len(header), KEEP_COLS):
                            header.append(HEADER_DEFAULT[i])
                    fout.write("# " + " ".join(header) + "\n")
                    wrote_header = True
                continue

            # Data line
            tokens = s.split()
            if len(tokens) < KEEP_COLS:
                # Skip malformed rows
                continue
            fout.write(" ".join(tokens[:KEEP_COLS]) + "\n")

    # If no header line was written, prepend a default one (with #)
    if not wrote_header:
        content = dst.read_text(encoding="utf-8")
        header_line = "# " + " ".join(HEADER_DEFAULT) + "\n"
        dst.write_text(header_line + content, encoding="utf-8")


if __name__ == "__main__":
    process_file(INPUT_PATH_02, OUTPUT_PATH_02)
    print(f"Saved: {OUTPUT_PATH_02}")
    process_file(INPUT_PATH_15, OUTPUT_PATH_15)
    print(f"Saved: {OUTPUT_PATH_15}")
    process_file(INPUT_PATH_21, OUTPUT_PATH_21)
    print(f"Saved: {OUTPUT_PATH_21}")
