#!/usr/bin/env python3
"""Convert the vendor DH116S coupling xlsx tables to the bundled CSVs in data/.

The vendor manual (§3.4 attachment, `DH116S被动与主动关系.7z`) ships per-finger
lookup tables relating the ACTIVE drive/linkage angle (连杆角位移) to the
passive knuckle angle (指节角位移) and the knuckle angle to the fingertip angle
(指尖角位移).  This script is a build-time tool: run it once against the
extracted vendor folder to (re)generate `data/coupling_<finger>_<pair>.csv`.
The generated CSVs are committed so the runtime package needs no openpyxl.

Usage:
    python3 extract_coupling.py /path/to/DH116S被动与主动关系 [--out ../data]

Notes:
- In the 指节-指尖 workbooks the second column header says 连杆角位移 but the
  values are the knuckle angle (verified: they match column 3 of the
  连杆-指节 workbook row-for-row) — a vendor copy/paste slip.
- The thumb (拇指) only ships a knuckle→fingertip table.
- Rows are deduplicated on the input angle and sorted so the CSV is strictly
  monotonic in the input column (required by the interpolators in coupling.py).
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

FINGER_DIRS = {
    "thumb": "拇指",
    "index": "食指",
    "middle": "中指",
    "ring": "无名指",
    "pinky": "小拇指",
}

PAIR_KEYWORDS = {
    "linkage_to_knuckle": "连杆-指节",
    "knuckle_to_fingertip": "指节-指尖",
}


def extract_pairs(xlsx_path: Path) -> list[tuple[float, float]]:
    import openpyxl  # build-time only

    wb = openpyxl.load_workbook(xlsx_path, read_only=True, data_only=True)
    ws = wb["Sheet1"]
    pairs: dict[float, float] = {}
    for row in ws.iter_rows(values_only=True):
        if row is None or len(row) < 3:
            continue
        x, y = row[1], row[2]
        if isinstance(x, (int, float)) and isinstance(y, (int, float)):
            pairs[float(x)] = float(y)
    if not pairs:
        raise ValueError(f"no numeric rows found in {xlsx_path}")
    return sorted(pairs.items())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("vendor_dir", type=Path, help="extracted DH116S被动与主动关系 folder")
    ap.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "data",
        help="output data directory (default: ../data)",
    )
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    written = 0
    for finger, cn_dir in FINGER_DIRS.items():
        finger_dir = args.vendor_dir / cn_dir
        if not finger_dir.is_dir():
            print(f"warning: missing vendor dir {finger_dir}", file=sys.stderr)
            continue
        for pair, keyword in PAIR_KEYWORDS.items():
            matches = sorted(finger_dir.glob(f"*{keyword}*.xlsx"))
            if not matches:
                if not (finger == "thumb" and pair == "linkage_to_knuckle"):
                    print(f"warning: no {keyword} xlsx for {finger}", file=sys.stderr)
                continue
            rows = extract_pairs(matches[0])
            out_path = args.out / f"coupling_{finger}_{pair}.csv"
            with out_path.open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["input_deg", "output_deg"])
                for x, y in rows:
                    w.writerow([f"{x:.6f}", f"{y:.6f}"])
            print(f"wrote {out_path} ({len(rows)} rows)")
            written += 1
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main())
