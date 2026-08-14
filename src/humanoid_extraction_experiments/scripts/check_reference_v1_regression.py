#!/usr/bin/env python3
"""Freeze the two preserved v1 continuation failures as a static regression gate."""

import argparse
import csv
from pathlib import Path


def rows(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def close(actual: str, expected: float, tolerance: float = 1e-9) -> bool:
    return abs(float(actual) - expected) <= tolerance


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--validation-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    forward = rows(args.validation_dir / "stage_constrained_reference_trials.csv")
    reverse = rows(args.validation_dir / "grasp_seeded_reference_trials.csv")
    forward_matches = [r for r in forward if close(r["initial_lift"], 0.30)
                       and r["failure_stage"] == "VERTICAL_DESCENT"
                       and r["failure_reason"] == "CONTINUOUS_IK_FAILURE"
                       and int(r["failure_waypoint"]) == 30
                       and close(r["cartesian_fraction"], 29.0 / 34.0)]
    reverse_matches = [r for r in reverse if close(r["initial_lift"], 0.35)
                       and r["failure_stage"] == "REVERSE_GRASP_TO_TOP_IK"
                       and r["failure_reason"] == "CONTINUOUS_IK_FAILURE"
                       and int(r["failure_waypoint"]) == 155
                       and close(r["reverse_fraction"], 11.0 / 12.0)
                       and close(r["accepted_spacing_m"], 0.001)]
    passed = len(forward_matches) == 2 and len(reverse_matches) == 2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        "# v1 reference regression fixture\n\n"
        f"- status: `{'PASS' if passed else 'FAIL'}`\n"
        f"- forward Lift 0.30 matches: {len(forward_matches)} (expected 2)\n"
        f"- reverse Lift 0.35 matches: {len(reverse_matches)} (expected 2)\n"
        "- forward invariant: waypoint 30, fraction 29/34 (0.852941...)\n"
        "- reverse invariant: waypoint 155, fraction 11/12 (0.916667...), spacing 0.001 m\n"
        "- This is a preserved-artifact check; it does not execute ROS or planning.\n",
        encoding="utf-8",
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
