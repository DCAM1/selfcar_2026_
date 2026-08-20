#!/usr/bin/env python3
"""Remove isolated zero-width sliver lanelets that cannot participate in routes."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from lxml import etree
from shapely.geometry import Polygon


def points(lanelet: etree._Element, tag: str) -> list[tuple[float, float]]:
    return [
        (float(point.findtext("x")), float(point.findtext("y")))
        for point in lanelet.find(tag).findall("point")
    ]


def clean(input_path: Path, output_path: Path, report_path: Path) -> None:
    tree = etree.parse(str(input_path), etree.XMLParser(huge_tree=True))
    root = tree.getroot()
    removed: list[dict[str, object]] = []
    removed_ids: set[str] = set()

    for lanelet in list(root.findall("lanelet")):
        if lanelet.findall("predecessor") or lanelet.findall("successor"):
            continue
        left = points(lanelet, "leftBound")
        right = points(lanelet, "rightBound")
        polygon = Polygon(left + right[::-1])
        start_width = math.dist(left[0], right[0])
        end_width = math.dist(left[-1], right[-1])
        if polygon.area >= 0.05 or start_width >= 1.0e-6 or end_width >= 1.0e-6:
            continue
        lanelet_id = lanelet.get("id")
        removed_ids.add(lanelet_id)
        removed.append(
            {
                "id": lanelet_id,
                "type": lanelet.findtext("laneletType", default=""),
                "area_m2": polygon.area,
                "start_width_m": start_width,
                "end_width_m": end_width,
                "reason": "isolated lanelet with zero width at both ends",
            }
        )
        root.remove(lanelet)

    removed_references: list[dict[str, str]] = []
    for lanelet in root.findall("lanelet"):
        for child in list(lanelet):
            if child.get("ref") in removed_ids:
                removed_references.append(
                    {
                        "owner_lanelet": lanelet.get("id"),
                        "element": child.tag,
                        "removed_ref": child.get("ref"),
                    }
                )
                lanelet.remove(child)

    tree.write(
        str(output_path),
        xml_declaration=True,
        encoding="UTF-8",
        pretty_print=True,
    )
    report_path.write_text(
        json.dumps(
            {
                "input": str(input_path),
                "output": str(output_path),
                "removed_count": len(removed),
                "removed_lanelets": removed,
                "removed_references": removed_references,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    clean(args.input, args.output, args.report)


if __name__ == "__main__":
    main()
