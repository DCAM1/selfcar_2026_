#!/usr/bin/env python3
"""Repair lanelets that become invalid after Autoware's float32 conversion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from lxml import etree
import numpy as np
from shapely.geometry import Polygon
from shapely.validation import explain_validity

from repair_commonroad_lanelets import read_boundary, repair_bounds, write_boundary


def quantize(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(float(np.float32(x)), float(np.float32(y))) for x, y in points]


def repair(input_path: Path, output_path: Path, report_path: Path) -> None:
    tree = etree.parse(str(input_path), etree.XMLParser(huge_tree=True))
    repaired: list[dict[str, object]] = []

    for lanelet in tree.getroot().findall("lanelet"):
        left = quantize(read_boundary(lanelet, "leftBound"))
        right = quantize(read_boundary(lanelet, "rightBound"))
        quantized_polygon = Polygon(left + right[::-1])
        if quantized_polygon.is_valid and quantized_polygon.area > 1.0e-8:
            continue

        fixed_left, fixed_right, method, score = repair_bounds(left, right)
        fixed_left = quantize(fixed_left)
        fixed_right = quantize(fixed_right)
        fixed_polygon = Polygon(fixed_left + fixed_right[::-1])
        if not fixed_polygon.is_valid or fixed_polygon.area <= 1.0e-8:
            raise RuntimeError(
                f"lanelet {lanelet.get('id')} is still invalid after float32 repair: "
                f"{explain_validity(fixed_polygon)}"
            )
        write_boundary(lanelet.find("leftBound"), fixed_left)
        write_boundary(lanelet.find("rightBound"), fixed_right)
        repaired.append(
            {
                "id": lanelet.get("id"),
                "type": lanelet.findtext("laneletType", default=""),
                "reason_before": explain_validity(quantized_polygon),
                "area_before_m2": quantized_polygon.area,
                "area_after_m2": fixed_polygon.area,
                "method": method,
                "score": score,
            }
        )

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
                "repaired_count": len(repaired),
                "repaired_lanelets": repaired,
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
    repair(args.input, args.output, args.report)


if __name__ == "__main__":
    main()
