# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Rewrite old v2 hand names to the current ones in your own files.

The mapping is v2/name_changes.csv. Pick where your old names come from:

  --source main           orcahand_description main before the rename, and the
                          hash-named files on release/15-10-26: names such as
                          I-FingerTipAssembly_ec49c16c and MJCF joints such as right_i-mcp
  --source fb-split-mjcf  the fb/split-mjcf-scene-positioning names, e.g.
                          right_index_fingertip_assembly

The *_sem files on release/15-10-26 (right_palm, right_index_mp, ...) are not
covered.

Names that were removed (the fingertip anchors) have no replacement; they are
reported and left as they are. All names are replaced in one pass, so a rename
never feeds into another (main's MJCF body right_tower becomes right_base while
its tower part becomes right_tower). Because right_tower is both an old and a
current name, run the script once per file: a file that already contains
current names is skipped.

Usage:
    python utils/migrate_v2_names.py --source main --side right FILE [FILE ...]
    python utils/migrate_v2_names.py --source main --side right --check FILE ...
"""

import argparse
import csv
import re
import sys
from pathlib import Path

CSV_PATH = Path(__file__).resolve().parent.parent / "v2" / "name_changes.csv"
NAME_CHAR = r"[\w-]"  # names are made of letters, digits, "_" and "-"


def load_rows(source: str, side: str) -> list:
    with open(CSV_PATH, newline="") as f:
        return [row for row in csv.DictReader(f) if row["source"] == source and row["side"] == side]


def whole_names(names) -> re.Pattern:
    """Match any of the names, but only as a whole name, longest first."""
    alternatives = "|".join(map(re.escape, sorted(names, key=len, reverse=True)))
    return re.compile(rf"(?<!{NAME_CHAR})({alternatives})(?!{NAME_CHAR})")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--source", required=True, choices=["main", "fb-split-mjcf"])
    parser.add_argument("--side", required=True, choices=["right", "left"])
    parser.add_argument("--check", action="store_true", help="report what would change; write nothing")
    parser.add_argument("files", nargs="+", type=Path)
    args = parser.parse_args(argv)

    rows = load_rows(args.source, args.side)
    mapping = {row["old"]: row["new"] for row in rows}
    pattern = whole_names(mapping)
    # A current part name that was never an old name marks an already migrated file.
    # Joint names can't: main's MJCF already called the wrist joint right_wrist.
    current_parts = {row["new"] for row in rows
                     if row["new"] and ("link" in row["kind"] or "body" in row["kind"])} - set(mapping)
    already_current = whole_names(current_parts)

    for path in args.files:
        text = path.read_text()
        if already_current.search(text):
            print(f"{path}: skipped, it already uses current names (mixed files need editing by hand)")
            continue
        found = [m.group(1) for m in pattern.finditer(text)]
        removed = sorted({name for name in found if not mapping[name]})
        renamed = sum(1 for name in found if mapping[name])
        if renamed and not args.check:
            path.write_text(pattern.sub(lambda m: mapping[m.group(1)] or m.group(1), text))
        verb = "would rename" if args.check else "renamed"
        print(f"{path}: {verb} {renamed} names")
        for name in removed:
            print(f"  {name}: removed in the current model, no replacement")
    return 0


if __name__ == "__main__":
    sys.exit(main())
