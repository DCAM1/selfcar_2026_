#!/usr/bin/env python3
"""Record the deterministic CommonRoad lanelet to exported OSM relation mapping."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from lxml import etree


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("commonroad", type=Path)
    parser.add_argument("osm", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    scenario, _ = CommonRoadFileReader(args.commonroad).open()
    commonroad_ids = [str(ll.lanelet_id) for ll in scenario.lanelet_network.lanelets]
    osm_ids: list[str] = []
    for _event, element in etree.iterparse(str(args.osm), events=("end",), tag="relation"):
        tags = {tag.get("k"): tag.get("v") for tag in element.findall("tag")}
        if tags.get("type") == "lanelet":
            osm_ids.append(element.get("id"))
        element.clear()
    if len(commonroad_ids) != len(osm_ids):
        raise RuntimeError(
            f"lanelet count mismatch: CommonRoad={len(commonroad_ids)}, OSM={len(osm_ids)}"
        )
    report = {
        "commonroad": str(args.commonroad),
        "osm": str(args.osm),
        "count": len(osm_ids),
        "commonroad_to_osm": dict(zip(commonroad_ids, osm_ids)),
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
