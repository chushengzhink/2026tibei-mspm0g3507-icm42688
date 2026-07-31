#!/usr/bin/env python3
"""Validate Q3 map CSV and generate q3_ball_profile_generated.h."""

from __future__ import annotations

import argparse
import csv
import io
import math
from dataclasses import dataclass
from pathlib import Path


FIELDS = (
    "position_cm",
    "balance_us",
    "roll_plus_us",
    "roll_minus_us",
    "break_plus_us",
    "break_minus_us",
    "accel_plus_cm_s2",
    "accel_minus_cm_s2",
    "valid_mask",
)
POSITIONS = tuple(float(value) for value in range(-6, 7))


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class Point:
    position_cm: float
    balance_us: float
    roll_plus_us: float
    roll_minus_us: float
    break_plus_us: float
    break_minus_us: float
    accel_plus_cm_s2: float
    accel_minus_cm_s2: float
    valid_mask: int


def _csv_region(text: str) -> str:
    lines = text.splitlines()
    header_index = next(
        (index for index, line in enumerate(lines)
         if line.strip() == ",".join(FIELDS)),
        None,
    )
    if header_index is None:
        raise ProfileError("Q3 map CSV header was not found")
    rows = [lines[header_index]]
    for line in lines[header_index + 1 :]:
        if line.strip() == "Q3_MAP_END":
            break
        if line.strip():
            rows.append(line)
    return "\n".join(rows)


def load_points(path: Path) -> list[Point]:
    region = _csv_region(path.read_text(encoding="utf-8-sig"))
    reader = csv.DictReader(io.StringIO(region))
    if tuple(reader.fieldnames or ()) != FIELDS:
        raise ProfileError("Q3 map CSV columns do not match the firmware")
    points: list[Point] = []
    try:
        for row in reader:
            points.append(
                Point(
                    *(float(row[name]) for name in FIELDS[:-1]),
                    int(row["valid_mask"], 0),
                )
            )
    except (TypeError, ValueError) as exc:
        raise ProfileError(f"invalid numeric field: {exc}") from exc
    validate_points(points)
    return points


def validate_points(points: list[Point]) -> None:
    if len(points) != len(POSITIONS):
        raise ProfileError("map must contain exactly 13 rows from -6 to +6 cm")
    for index, (point, expected) in enumerate(zip(points, POSITIONS)):
        values = (
            point.position_cm,
            point.balance_us,
            point.roll_plus_us,
            point.roll_minus_us,
            point.break_plus_us,
            point.break_minus_us,
            point.accel_plus_cm_s2,
            point.accel_minus_cm_s2,
        )
        if not all(math.isfinite(value) for value in values):
            raise ProfileError(f"row {index}: non-finite value")
        if abs(point.position_cm - expected) > 0.05:
            raise ProfileError(
                f"row {index}: expected {expected:+.0f} cm, "
                f"got {point.position_cm:+.3f} cm"
            )
        if point.valid_mask & 0x03 != 0x03:
            raise ProfileError(
                f"{expected:+.0f} cm: both travel directions were not observed"
            )
        if not -60.0 <= point.balance_us <= 60.0:
            raise ProfileError(f"{expected:+.0f} cm: balance exceeds +/-60 us")
        if not (5.0 <= point.roll_plus_us <= point.break_plus_us <= 250.0):
            raise ProfileError(f"{expected:+.0f} cm: invalid +cm thresholds")
        if not (5.0 <= point.roll_minus_us <= point.break_minus_us <= 250.0):
            raise ProfileError(f"{expected:+.0f} cm: invalid -cm thresholds")
        if not (1.0 <= point.accel_plus_cm_s2 <= 80.0):
            raise ProfileError(f"{expected:+.0f} cm: invalid +cm acceleration")
        if not (1.0 <= point.accel_minus_cm_s2 <= 80.0):
            raise ProfileError(f"{expected:+.0f} cm: invalid -cm acceleration")
        if index:
            previous = points[index - 1]
            for name in (
                "balance_us",
                "roll_plus_us",
                "roll_minus_us",
                "break_plus_us",
                "break_minus_us",
            ):
                if abs(getattr(point, name) - getattr(previous, name)) > 60.0:
                    raise ProfileError(
                        f"{expected:+.0f} cm: {name} jumps by more than 60 us"
                    )


def normalized_points(points: list[Point]) -> list[Point]:
    result: list[Point] = []
    for point in points:
        inferred_balance = point.balance_us + 0.5 * (
            point.roll_plus_us - point.roll_minus_us
        )
        result.append(
            Point(
                point.position_cm,
                max(-60.0, min(60.0, inferred_balance)),
                point.roll_plus_us,
                point.roll_minus_us,
                point.break_plus_us,
                point.break_minus_us,
                point.accel_plus_cm_s2,
                point.accel_minus_cm_s2,
                point.valid_mask,
            )
        )
    return result


def render_header(
    points: list[Point], axis_sign: int, neutral_us: float, probe_cm: float
) -> str:
    if axis_sign not in (-1, 1):
        raise ProfileError("axis sign must be -1 or +1")
    if not 1300.0 <= neutral_us <= 1700.0:
        raise ProfileError("neutral pulse must stay within 1300--1700 us")
    if not 0.05 <= probe_cm <= 1.0:
        raise ProfileError("probe displacement must be 0.05--1.0 cm")
    points = normalized_points(points)
    rows = []
    for point in points:
        rows.append(
            "    {{ {:+.1f}f, {:.1f}f, {:.1f}f, {:.1f}f, {:.1f}f, "
            "{:.1f}f, {:.2f}f, {:.2f}f }}".format(
                point.position_cm,
                point.balance_us,
                point.roll_plus_us,
                point.roll_minus_us,
                point.break_plus_us,
                point.break_minus_us,
                point.accel_plus_cm_s2,
                point.accel_minus_cm_s2,
            )
        )
    joined = ", \\\n".join(rows)
    return f"""#ifndef Q3_BALL_PROFILE_GENERATED_H
#define Q3_BALL_PROFILE_GENERATED_H

/* Generated by tools/q3_fit_profile.py. Do not hand-edit measured values. */
#define Q3_PROFILE_GENERATED_AXIS_SIGN ({axis_sign})
#define Q3_PROFILE_GENERATED_NEUTRAL_US ({neutral_us:.1f}f)
#define Q3_PROFILE_GENERATED_PROBE_CM ({probe_cm:.2f}f)
#define Q3_PROFILE_GENERATED_POINTS {{ \\
{joined} \\
}}

#endif
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--axis-sign", type=int, default=-1)
    parser.add_argument("--neutral-us", type=float, default=1525.0)
    parser.add_argument("--probe-cm", type=float, default=0.25)
    args = parser.parse_args()
    try:
        points = load_points(args.input)
        header = render_header(
            points, args.axis_sign, args.neutral_us, args.probe_cm
        )
    except ProfileError as exc:
        parser.error(str(exc))
    args.output.write_text(header, encoding="ascii", newline="\n")
    print(f"generated {args.output} from {len(points)} validated map points")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
