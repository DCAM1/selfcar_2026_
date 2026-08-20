#!/usr/bin/env python3
"""Feed OSM endpoint snapping back into CommonRoad for float32 robustness."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from lxml import etree
import numpy as np
from shapely.geometry import Polygon
from shapely.validation import explain_validity

from repair_commonroad_lanelets import read_boundary, repair_bounds, write_boundary


def quantize(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(float(np.float32(x)), float(np.float32(y))) for x, y in points]


def orient_like(
    points: list[tuple[float, float]], reference: list[tuple[float, float]]
) -> list[tuple[float, float]]:
    direct = math.dist(points[0], reference[0]) + math.dist(points[-1], reference[-1])
    reverse = math.dist(points[-1], reference[0]) + math.dist(points[0], reference[-1])
    return points if direct <= reverse else points[::-1]


def repair(
    commonroad_path: Path, osm_path: Path, output_path: Path, report_path: Path
) -> None:
    cr_tree = etree.parse(str(commonroad_path), etree.XMLParser(huge_tree=True))
    cr_lanelets = cr_tree.getroot().findall("lanelet")
    osm_root = etree.parse(str(osm_path), etree.XMLParser(huge_tree=True)).getroot()

    nodes: dict[str, tuple[float, float]] = {}
    for node in osm_root.findall("node"):
        tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
        nodes[node.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
    ways = {
        way.get("id"): [nodes[node.get("ref")] for node in way.findall("nd")]
        for way in osm_root.findall("way")
    }
    osm_lanelets = []
    for relation in osm_root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("type") == "lanelet":
            osm_lanelets.append(relation)
    if len(cr_lanelets) != len(osm_lanelets):
        raise RuntimeError(
            f"lanelet order mismatch: CommonRoad={len(cr_lanelets)}, OSM={len(osm_lanelets)}"
        )

    repaired: list[dict[str, object]] = []
    for cr_lanelet, osm_relation in zip(cr_lanelets, osm_lanelets):
        members = {
            member.get("role"): member.get("ref")
            for member in osm_relation.findall("member")
            if member.get("type") == "way"
        }
        cr_left = read_boundary(cr_lanelet, "leftBound")
        cr_right = read_boundary(cr_lanelet, "rightBound")
        osm_left = orient_like(ways[members["left"]], cr_left)
        osm_right = orient_like(ways[members["right"]], cr_right)
        quantized_left = quantize(osm_left)
        quantized_right = quantize(osm_right)
        polygon = Polygon(quantized_left + quantized_right[::-1])
        if polygon.is_valid and polygon.area > 1.0e-8:
            continue

        fixed_left, fixed_right, method, score = repair_bounds(
            quantized_left, quantized_right
        )
        fixed_left = quantize(fixed_left)
        fixed_right = quantize(fixed_right)
        fixed_polygon = Polygon(fixed_left + fixed_right[::-1])
        if not fixed_polygon.is_valid or fixed_polygon.area <= 1.0e-8:
            raise RuntimeError(
                f"lanelet {cr_lanelet.get('id')} remains invalid: "
                f"{explain_validity(fixed_polygon)}"
            )
        write_boundary(cr_lanelet.find("leftBound"), fixed_left)
        write_boundary(cr_lanelet.find("rightBound"), fixed_right)
        repaired.append(
            {
                "commonroad_id": cr_lanelet.get("id"),
                "osm_relation_id": osm_relation.get("id"),
                "reason_before": explain_validity(polygon),
                "area_before_m2": polygon.area,
                "area_after_m2": fixed_polygon.area,
                "method": method,
                "score": score,
            }
        )

    cr_tree.write(
        str(output_path),
        xml_declaration=True,
        encoding="UTF-8",
        pretty_print=True,
    )
    report_path.write_text(
        json.dumps(
            {
                "commonroad_input": str(commonroad_path),
                "osm_input": str(osm_path),
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
    parser.add_argument("commonroad", type=Path)
    parser.add_argument("osm", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    repair(args.commonroad, args.osm, args.output, args.report)


if __name__ == "__main__":
    main()
