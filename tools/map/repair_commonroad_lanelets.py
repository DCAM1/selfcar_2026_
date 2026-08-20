#!/usr/bin/env python3
"""Repair self-intersecting CommonRoad lanelet boundaries.

The OpenDRIVE conversion can produce a valid-looking centerline while one of
the sampled boundaries folds back on itself at a junction or lane-section
transition.  This utility repairs only those lanelets.  It uses the polygon's
valid exterior as a geometric guardrail, then restores two ordered boundary
polylines so the normal CRDesigner Lanelet2 exporter can consume the result.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
from typing import Iterable, Sequence

from lxml import etree
from shapely.geometry import LineString, Polygon


def point_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def clean_points(points: Iterable[Sequence[float]]) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    for point in points:
        value = (float(point[0]), float(point[1]))
        if not result or point_distance(result[-1], value) > 1.0e-9:
            result.append(value)
    return result


def resample_points(
    points: list[tuple[float, float]], count: int
) -> list[tuple[float, float]]:
    """Resample a polyline at equally spaced arc-length positions."""
    points = clean_points(points)
    if len(points) < 2 or count <= 2:
        return [points[0], points[-1]]
    if len(points) == count:
        return points

    distances = [0.0]
    for first, second in zip(points, points[1:]):
        distances.append(distances[-1] + point_distance(first, second))
    total = distances[-1]
    if total <= 1.0e-12:
        return [points[0] for _ in range(count)]

    result: list[tuple[float, float]] = []
    segment = 0
    for sample in [total * i / (count - 1) for i in range(count)]:
        while segment < len(distances) - 2 and distances[segment + 1] < sample:
            segment += 1
        length = distances[segment + 1] - distances[segment]
        ratio = 0.0 if length <= 1.0e-12 else (sample - distances[segment]) / length
        first = points[segment]
        second = points[segment + 1]
        result.append(
            (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
        )
    return result


def ring_arc(
    ring: list[tuple[float, float]], start: int, end: int, forward: bool
) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    index = start
    for _ in range(len(ring) + 1):
        result.append(ring[index])
        if index == end:
            break
        index = (index + 1) % len(ring) if forward else (index - 1) % len(ring)
    return result


def nearest_indices(
    ring: list[tuple[float, float]], point: Sequence[float], count: int = 5
) -> list[int]:
    return sorted(range(len(ring)), key=lambda i: point_distance(ring[i], point))[:count]


def write_boundary(boundary: etree._Element, points: list[tuple[float, float]]) -> None:
    for point in list(boundary.findall("point")):
        boundary.remove(point)

    insert_at = len(boundary)
    for index, child in enumerate(boundary):
        if child.tag != "point":
            insert_at = index
            break

    for offset, (x, y) in enumerate(points):
        point = etree.Element("point")
        etree.SubElement(point, "x").text = f"{x:.10f}"
        etree.SubElement(point, "y").text = f"{y:.10f}"
        boundary.insert(insert_at + offset, point)


def read_boundary(lanelet: etree._Element, tag: str) -> list[tuple[float, float]]:
    bound = lanelet.find(tag)
    if bound is None:
        raise ValueError(f"lanelet {lanelet.get('id')} has no {tag}")
    return [
        (float(point.findtext("x")), float(point.findtext("y")))
        for point in bound.findall("point")
    ]


def repair_bounds(
    left: list[tuple[float, float]], right: list[tuple[float, float]]
) -> tuple[list[tuple[float, float]], list[tuple[float, float]], str, float]:
    left = clean_points(left)
    right = clean_points(right)
    original = Polygon(left + right[::-1])

    # A closed boundary pair is a loop lanelet.  Removing the duplicated last
    # sample converts it to a valid open lanelet without moving its route.
    if (
        len(left) > 3
        and len(right) > 3
        and point_distance(left[0], left[-1]) < 1.0e-6
        and point_distance(right[0], right[-1]) < 1.0e-6
    ):
        trimmed_left = left[:-1]
        trimmed_right = right[:-1]
        trimmed_polygon = Polygon(trimmed_left + trimmed_right[::-1])
        if trimmed_polygon.is_valid and trimmed_polygon.area > 1.0e-8:
            return trimmed_left, trimmed_right, "trim_closed_loop", 0.0

    repaired = original.buffer(0)
    if repaired.geom_type == "MultiPolygon":
        repaired = max(repaired.geoms, key=lambda polygon: polygon.area)
    if repaired.geom_type != "Polygon" or repaired.area <= 1.0e-8:
        raise ValueError("buffer(0) did not produce a usable polygon")

    ring = list(repaired.exterior.coords)[:-1]
    left_start, right_start = left[0], right[0]
    left_end, right_end = left[-1], right[-1]

    # At a lane split/join the source can legitimately start or end at zero
    # width.  Make that explicit so both boundaries meet at one point.
    if point_distance(left_start, right_start) < 0.02:
        midpoint = (
            (left_start[0] + right_start[0]) / 2.0,
            (left_start[1] + right_start[1]) / 2.0,
        )
        left_start = right_start = midpoint
    if point_distance(left_end, right_end) < 0.02:
        midpoint = (
            (left_end[0] + right_end[0]) / 2.0,
            (left_end[1] + right_end[1]) / 2.0,
        )
        left_end = right_end = midpoint

    def find_candidates(count: int):
        endpoint_candidates = [
            nearest_indices(ring, left_start, count),
            nearest_indices(ring, left_end, count),
            nearest_indices(ring, right_start, count),
            nearest_indices(ring, right_end, count),
        ]
        candidates = []
        for indices in itertools.product(*endpoint_candidates):
            for left_forward, right_forward in itertools.product((True, False), repeat=2):
                repaired_left = clean_points(
                    ring_arc(ring, indices[0], indices[1], left_forward)
                )
                repaired_right = clean_points(
                    ring_arc(ring, indices[2], indices[3], right_forward)
                )
                if len(repaired_left) < 2 or len(repaired_right) < 2:
                    continue

                repaired_left[0] = left_start
                repaired_left[-1] = left_end
                repaired_right[0] = right_start
                repaired_right[-1] = right_end
                repaired_left = clean_points(repaired_left)
                repaired_right = clean_points(repaired_right)

                target_count = max(len(left), len(right), len(repaired_left), len(repaired_right))
                repaired_left = resample_points(repaired_left, target_count)
                repaired_right = resample_points(repaired_right, target_count)

                polygon = Polygon(repaired_left + repaired_right[::-1])
                if not polygon.is_valid or polygon.area <= 1.0e-8:
                    continue
                if not LineString(repaired_left).is_simple:
                    continue
                if not LineString(repaired_right).is_simple:
                    continue

                score = LineString(repaired_left).hausdorff_distance(LineString(left))
                score += LineString(repaired_right).hausdorff_distance(LineString(right))
                score += 0.01 * abs(polygon.area - repaired.area) / max(repaired.area, 1.0e-8)
                candidates.append((score, repaired_left, repaired_right))
        return candidates

    # Most invalid polygons have an unambiguous nearest exterior vertex.  This
    # cheap pass handles the normal case; the wider search is only needed for
    # tiny zero-width lanelets whose endpoints collapse to the same exterior
    # vertex.
    candidates = find_candidates(1)
    if not candidates:
        candidates = find_candidates(5)

    if not candidates:
        raise ValueError("could not find two simple boundary arcs")

    score, repaired_left, repaired_right = min(candidates, key=lambda item: item[0])
    return repaired_left, repaired_right, "buffer_exterior", score


def repair_file(input_path: Path, output_path: Path, report_path: Path) -> None:
    tree = etree.parse(str(input_path), etree.XMLParser(huge_tree=True, remove_blank_text=False))
    repaired_count = 0
    failed: list[str] = []
    report = []

    for lanelet in tree.getroot().findall("lanelet"):
        left = read_boundary(lanelet, "leftBound")
        right = read_boundary(lanelet, "rightBound")
        original_polygon = Polygon(left + right[::-1])
        if original_polygon.is_valid:
            continue

        lanelet_id = lanelet.get("id", "")
        try:
            fixed_left, fixed_right, method, score = repair_bounds(left, right)
            fixed_polygon = Polygon(fixed_left + fixed_right[::-1])
            if not fixed_polygon.is_valid:
                raise ValueError("repair result is still invalid")
            write_boundary(lanelet.find("leftBound"), fixed_left)
            write_boundary(lanelet.find("rightBound"), fixed_right)
            repaired_count += 1
            report.append(
                {
                    "id": lanelet_id,
                    "type": lanelet.findtext("laneletType", default=""),
                    "method": method,
                    "score": score,
                    "area_before": original_polygon.area,
                    "area_after": fixed_polygon.area,
                }
            )
        except Exception as error:  # pragma: no cover - retained in report for field maps
            failed.append(lanelet_id)
            report.append(
                {
                    "id": lanelet_id,
                    "type": lanelet.findtext("laneletType", default=""),
                    "method": "failed",
                    "error": str(error),
                }
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(output_path), xml_declaration=True, encoding="UTF-8", pretty_print=True)
    report_path.write_text(
        json.dumps(
            {
                "input": str(input_path),
                "output": str(output_path),
                "invalid_before": len(report),
                "repaired": repaired_count,
                "failed": failed,
                "lanelets": report,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    if failed:
        raise RuntimeError(f"failed to repair lanelets: {', '.join(failed)}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    repair_file(args.input, args.output, args.report)


if __name__ == "__main__":
    main()
