#!/usr/bin/env python3
"""Validate Lanelet2 geometry and create a whole-map diagnostic image."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from lxml import etree
import numpy as np
from shapely.geometry import Polygon


def validate(input_path: Path, report_path: Path, image_path: Path | None) -> None:
    nodes: dict[str, tuple[float, float]] = {}
    ways: dict[str, list[str]] = {}
    way_tags: dict[str, dict[str, str]] = {}
    lanelet_relations: list[tuple[str, dict[str, str], dict[str, str]]] = []
    meta_info: dict[str, str] = {}

    for _event, element in etree.iterparse(
        str(input_path), events=("end",), tag=("MetaInfo", "node", "way", "relation")
    ):
        if element.tag == "MetaInfo":
            meta_info = dict(element.attrib)
        elif element.tag == "node":
            tags = {tag.get("k"): tag.get("v") for tag in element.findall("tag")}
            if "local_x" in tags and "local_y" in tags:
                nodes[element.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
            else:
                nodes[element.get("id")] = (
                    float(element.get("lon", "0")),
                    float(element.get("lat", "0")),
                )
        elif element.tag == "way":
            way_id = element.get("id")
            ways[way_id] = [nd.get("ref") for nd in element.findall("nd")]
            way_tags[way_id] = {
                tag.get("k", ""): tag.get("v", "") for tag in element.findall("tag")
            }
        else:
            tags = {tag.get("k"): tag.get("v") for tag in element.findall("tag")}
            if tags.get("type") == "lanelet":
                members = {
                    member.get("role"): member.get("ref")
                    for member in element.findall("member")
                    if member.get("type") == "way"
                }
                lanelet_relations.append((element.get("id"), members, tags))
        element.clear()

    missing_node_references: list[dict[str, str]] = []
    zero_length_segments: list[dict[str, object]] = []
    degenerate_ways: list[str] = []
    for way_id, refs in ways.items():
        if len(refs) < 2 or len(set(refs)) < 2:
            degenerate_ways.append(way_id)
        for index, (first, second) in enumerate(zip(refs, refs[1:])):
            if first not in nodes or second not in nodes:
                missing_node_references.append(
                    {"way": way_id, "first": first, "second": second}
                )
                continue
            if math.dist(nodes[first], nodes[second]) <= 1.0e-9:
                zero_length_segments.append(
                    {"way": way_id, "segment_index": index, "nodes": [first, second]}
                )

    invalid_details: list[dict[str, object]] = []
    float32_invalid_details: list[dict[str, object]] = []
    missing_way_references: list[dict[str, str]] = []
    lanelet_areas: list[float] = []
    lanelet_lines: list[list[tuple[float, float]]] = []
    invalid_lines: list[list[tuple[float, float]]] = []
    bounds = [math.inf, math.inf, -math.inf, -math.inf]

    for relation_id, members, _tags in lanelet_relations:
        left_id, right_id = members.get("left"), members.get("right")
        if left_id not in ways or right_id not in ways:
            missing_way_references.append(
                {"relation": relation_id, "left": left_id, "right": right_id}
            )
            continue
        try:
            left = [nodes[ref] for ref in ways[left_id]]
            right = [nodes[ref] for ref in ways[right_id]]
        except KeyError as error:
            missing_way_references.append(
                {"relation": relation_id, "missing_node": str(error)}
            )
            continue
        if len(left) < 2 or len(right) < 2:
            invalid_details.append(
                {"relation": relation_id, "reason": "boundary has fewer than two nodes"}
            )
            continue

        direct = math.dist(left[0], right[0]) + math.dist(left[-1], right[-1])
        crossed = math.dist(left[0], right[-1]) + math.dist(left[-1], right[0])
        if crossed < direct:
            right = right[::-1]
        polygon = Polygon(left + right[::-1])
        lanelet_areas.append(polygon.area)
        line = left + right[::-1] + [left[0]]
        lanelet_lines.append(line)
        for x, y in line:
            bounds[0] = min(bounds[0], x)
            bounds[1] = min(bounds[1], y)
            bounds[2] = max(bounds[2], x)
            bounds[3] = max(bounds[3], y)
        if not polygon.is_valid or polygon.area <= 1.0e-8:
            invalid_details.append(
                {
                    "relation": relation_id,
                    "valid": polygon.is_valid,
                    "area_m2": polygon.area,
                }
            )
            invalid_lines.append(line)
        float32_polygon = Polygon(np.asarray(left + right[::-1], dtype=np.float32))
        if not float32_polygon.is_valid or float32_polygon.area <= 1.0e-8:
            float32_invalid_details.append(
                {
                    "relation": relation_id,
                    "valid": float32_polygon.is_valid,
                    "area_m2": float32_polygon.area,
                }
            )

    report = {
        "validated_at": datetime.now(timezone.utc).isoformat(),
        "file": str(input_path),
        "nodes": len(nodes),
        "ways": len(ways),
        "lanelets": len(lanelet_relations),
        "invalid_lanelet_polygons": len(invalid_details),
        "float32_invalid_lanelet_polygons": len(float32_invalid_details),
        "zero_length_way_segments": len(zero_length_segments),
        "degenerate_ways": degenerate_ways,
        "missing_node_references": missing_node_references,
        "missing_way_references": missing_way_references,
        "minimum_lanelet_area_m2": min(lanelet_areas) if lanelet_areas else None,
        "map_bounds_local_m": bounds if lanelet_lines else None,
        "meta_info": meta_info,
        "invalid_details": invalid_details,
        "float32_invalid_details": float32_invalid_details,
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    if image_path is not None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        figure, axis = plt.subplots(figsize=(16, 12), dpi=160)
        axis.add_collection(LineCollection(lanelet_lines, colors="#2a5b84", linewidths=0.16))
        if invalid_lines:
            axis.add_collection(LineCollection(invalid_lines, colors="red", linewidths=1.2))
        axis.autoscale()
        axis.set_aspect("equal", adjustable="box")
        axis.set_title(
            f"VTD Lanelet2 geometry: {len(lanelet_relations)} lanelets, "
            f"{len(invalid_details)} invalid polygons"
        )
        axis.set_xlabel("local x [m]")
        axis.set_ylabel("local y [m]")
        axis.grid(True, linewidth=0.2, alpha=0.35)
        figure.tight_layout()
        figure.savefig(image_path)
        plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--image", type=Path)
    args = parser.parse_args()
    validate(args.input, args.report, args.image)


if __name__ == "__main__":
    main()
