#!/usr/bin/env python3
"""Compare final exported Lanelet2 boundaries with their CommonRoad sources."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from lxml import etree
from shapely.geometry import LineString


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("commonroad", type=Path)
    parser.add_argument("osm", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    scenario, _ = CommonRoadFileReader(args.commonroad).open()
    lanelets = {str(lanelet.lanelet_id): lanelet for lanelet in scenario.lanelet_network.lanelets}
    root = etree.parse(str(args.osm), etree.XMLParser(huge_tree=True)).getroot()
    nodes = {}
    for node in root.findall("node"):
        tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
        nodes[node.get("id")] = (
            float(tags.get("local_x", node.get("lon", "0"))),
            float(tags.get("local_y", node.get("lat", "0"))),
        )
    ways = {
        way.get("id"): [nodes[nd.get("ref")] for nd in way.findall("nd")]
        for way in root.findall("way")
    }

    comparisons = []
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        source_id = tags.get("source_commonroad_id")
        if source_id is None:
            continue
        members = {
            member.get("role"): member.get("ref")
            for member in relation.findall("member")
            if member.get("type") == "way"
        }
        lanelet = lanelets[source_id]
        left_error = LineString(lanelet.left_vertices[:, :2]).hausdorff_distance(
            LineString(ways[members["left"]])
        )
        right_error = LineString(lanelet.right_vertices[:, :2]).hausdorff_distance(
            LineString(ways[members["right"]])
        )
        comparisons.append(
            {
                "error_m": max(left_error, right_error),
                "commonroad_id": source_id,
                "osm_relation_id": relation.get("id"),
                "left_error_m": left_error,
                "right_error_m": right_error,
            }
        )
    comparisons.sort(key=lambda item: item["error_m"], reverse=True)
    errors = sorted(item["error_m"] for item in comparisons)
    percentile_index = min(len(errors) - 1, int(0.95 * len(errors))) if errors else 0
    report = {
        "lanelets_compared": len(comparisons),
        "maximum_boundary_hausdorff_error_m": errors[-1] if errors else None,
        "p95_boundary_hausdorff_error_m": errors[percentile_index] if errors else None,
        "over_1_mm": sum(error > 0.001 for error in errors),
        "over_1_cm": sum(error > 0.01 for error in errors),
        "over_10_cm": sum(error > 0.1 for error in errors),
        "over_1_m": sum(error > 1.0 for error in errors),
        "top": comparisons[:30],
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
