# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Find old v2 hand names in your own files and rewrite them to the current names.

v2/name_changes.csv lists every old name with its new name. Choose where your
old names come from:

  --source main           orcahand_description main before the rename, and the
                          hash-named files on release/15-10-26, e.g. the URDF
                          link I-FingerTipAssembly_ec49c16c or the MJCF joint
                          right_i-mcp
  --source fb-split-mjcf  the fb/split-mjcf-scene-positioning branch, e.g.
                          right_index_fingertip_assembly

Not covered: the *_sem files on release/15-10-26 (right_palm, right_index_mp,
...) and names without the side prefix (orca_ui's bundle).

Usage:
    python utils/migrate_v2_names.py --source main --side right FILE [FILE ...]
    python utils/migrate_v2_names.py --source main --side right --write FILE ...

Without --write, the script writes nothing: it lists each rename per file and
exits with code 1 if any file still has old or removed names. With --write, the
script rewrites the files. Commit your files first, so the change can be
reviewed and undone. In both modes, the script exits with code 1 if it skipped
a file for you to check: one it can't read as text, one with the other hand's
or the other source's old names, one that mixes old and current names, or one
whose only old name is right_tower.

Rules:
- Give one hand's files per run. On main, 18 names (e.g.
  ForeArmStructure-Model_e18f2368) were the same in the right and left files.
  The script skips a file with old names that only the other hand used: a file
  of the wrong hand, or of both hands. A file with only the 18 shared names
  can't show its hand, so it gets --side's names and the script says so.
- Removed names (the fingertip anchors, unused mesh assets) have no new name.
  The script lists removed names and leaves them in the file.
- All names are replaced in one pass, so one rename never feeds into another.
  Example from main: the MJCF body right_tower becomes right_base, while the
  MJCF body right_TopTower-Model_4a80d30e becomes right_tower.
- right_tower is both an old and a current name. So a file is rewritten only
  if it has an old name that the script renames and that isn't a current name,
  and no current part name. A second run on the same file changes nothing.
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


def old_proof(rows) -> set:
    """Old names that show a file is old: renamed ones that aren't also a current name."""
    return {row["old"] for row in rows if row["new"]} - {row["new"] for row in rows}


def whole_names(names) -> re.Pattern:
    """Match any of the names, but only as a whole name, longest first."""
    alternatives = "|".join(map(re.escape, sorted(names, key=len, reverse=True)))
    return re.compile(rf"(?<!{NAME_CHAR})({alternatives})(?!{NAME_CHAR})")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--source", required=True, choices=["main", "fb-split-mjcf"])
    parser.add_argument("--side", required=True, choices=["right", "left"])
    parser.add_argument("--write", action="store_true", help="rewrite the files (without it, only report)")
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
    current_names = {row["new"] for row in rows if row["new"]}
    # Old names that only the other hand used.
    other_side = "left" if args.side == "right" else "right"
    other_rows = load_rows(args.source, other_side)
    shared = set(mapping) & {row["old"] for row in other_rows}
    other_pattern = whole_names(old_proof(other_rows) - shared)
    # Old names of the other source, either hand.
    other_source = "fb-split-mjcf" if args.source == "main" else "main"
    other_source_rows = load_rows(other_source, "right") + load_rows(other_source, "left")
    other_source_pattern = whole_names(old_proof(other_source_rows) - set(mapping))

    pending = False  # a file still has old or removed names
    check_by_hand = False  # a file was skipped for a person to look at
    for path in args.files:
        try:
            with open(path, encoding="utf-8", newline="") as f:
                text = f.read()
        except (OSError, UnicodeDecodeError) as error:
            print(f"{path}: skipped, can't read it as text ({error})")
            check_by_hand = True
            continue
        found = pattern.findall(text)
        removed = sorted({name for name in found if not mapping[name]})
        # Names that prove the file is old: ones this script renames, minus current names
        # (right_tower is both). Removed names don't count: they stay after a rewrite.
        old = {name for name in found if mapping[name]} - current_names
        pending = pending or bool(old or removed)
        other = sorted(set(other_pattern.findall(text)))
        if other:
            check_by_hand = True
            own = sorted(old - shared)
            if own:
                print(f"{path}: skipped, it has old names of both hands ({own[0]}, {other[0]}), edit it by hand")
            else:
                print(f"{path}: skipped, it has old {other_side} hand names ({other[0]}), use --side {other_side}")
            continue
        # A file with nothing to rename may hold the other source's names (fb-split files
        # already had some current names, e.g. right_carpals).
        other_source_names = [] if old else sorted(set(other_source_pattern.findall(text)))
        if other_source_names:
            check_by_hand = True
            print(f"{path}: nothing to rename, but it has --source {other_source} names "
                  f"({other_source_names[0]}), use --source {other_source}")
        elif already_current.search(text):
            if old:
                check_by_hand = True
                print(f"{path}: skipped, it mixes old and current names, edit it by hand")
            else:
                print(f"{path}: skipped, it already uses current names")
        elif not old:
            ambiguous = sorted(set(found) - set(removed))
            if ambiguous:
                check_by_hand = True
                print(f"{path}: skipped, {', '.join(ambiguous)} is an old and a current name, check by hand")
            else:
                print(f"{path}: nothing to rename")
        else:
            renamed = sum(1 for name in found if mapping[name])
            if args.write:
                with open(path, "w", encoding="utf-8", newline="") as f:
                    f.write(pattern.sub(lambda m: mapping[m.group(1)] or m.group(1), text))
            verb = "renamed" if args.write else "would rename"
            print(f"{path}: {verb} {renamed} names")
            if old <= shared:
                print(f"  its old names were the same for both hands: check it is a {args.side} hand file")
            for name in sorted({name for name in found if mapping[name]}):
                print(f"  {name} -> {mapping[name]}")
        for name in removed:
            print(f"  {name}: removed in the current model, no replacement")
    return 1 if check_by_hand or (pending and not args.write) else 0


if __name__ == "__main__":
    sys.exit(main())
