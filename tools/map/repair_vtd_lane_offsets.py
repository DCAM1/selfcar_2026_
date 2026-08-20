#!/usr/bin/env python3
"""Make implicit/missing zero OpenDRIVE records explicit.

OpenDRIVE defines the lane offset as zero before the first ``laneOffset``
record.  Some converters, including the CRDesigner release used for this
map, incorrectly select the final record when a road's first record starts
after s=0.  That shifts the entire earlier part of the road sideways by one
or more lane widths.  This utility preserves the source and writes a schema-
compatible copy with an explicit zero record at s=0 for affected roads.  It
also repairs the same structural defect for lane ``width``/``border`` records:
their first ``sOffset`` must start at zero inside a lane section.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from lxml import etree


ZERO = "0.0000000000000000e+00"


def repair(input_path: Path, output_path: Path, report_path: Path) -> None:
    parser = etree.XMLParser(huge_tree=True, remove_blank_text=False)
    tree = etree.parse(str(input_path), parser)
    repaired: list[dict[str, object]] = []
    repaired_lane_widths: list[dict[str, object]] = []
    normalized_offset_stations: list[dict[str, object]] = []

    for road in tree.getroot().findall("road"):
        lanes = road.find("lanes")
        if lanes is None:
            continue
        offsets = lanes.findall("laneOffset")
        if not offsets:
            continue

        first_s = float(offsets[0].get("s", "0"))
        if first_s <= 1.0e-9:
            continue

        zero_offset = etree.Element(
            "laneOffset", s=ZERO, a=ZERO, b=ZERO, c=ZERO, d=ZERO
        )
        lanes.insert(lanes.index(offsets[0]), zero_offset)
        repaired.append(
            {
                "road_id": road.get("id"),
                "road_name": road.get("name", ""),
                "first_original_s": first_s,
                "offset_records_before": len(offsets),
                "offset_records_after": len(offsets) + 1,
            }
        )

    # ROD sometimes emits records for the same station with differences near
    # machine precision.  CRDesigner compares them with exact equality and can
    # therefore select the wrong polynomial at a lane-section seam.  Snap such
    # clusters to the lane-section station while retaining document order (the
    # final record at a duplicated station is the effective OpenDRIVE record).
    station_tolerance = 1.0e-6
    for road in tree.getroot().findall("road"):
        lanes = road.find("lanes")
        if lanes is None:
            continue
        offsets = lanes.findall("laneOffset")
        if len(offsets) < 2:
            continue
        section_stations = [
            (float(section.get("s", "0")), section.get("s", ZERO))
            for section in lanes.findall("laneSection")
        ]
        ordered = sorted(offsets, key=lambda item: float(item.get("s", "0")))
        clusters: list[list[etree._Element]] = []
        for offset in ordered:
            if not clusters:
                clusters.append([offset])
                continue
            previous = float(clusters[-1][-1].get("s", "0"))
            current = float(offset.get("s", "0"))
            if abs(current - previous) <= station_tolerance:
                clusters[-1].append(offset)
            else:
                clusters.append([offset])

        for cluster in clusters:
            if len(cluster) < 2:
                continue
            values = [float(item.get("s", "0")) for item in cluster]
            station = sum(values) / len(values)
            section_match = min(
                section_stations,
                key=lambda item: abs(item[0] - station),
                default=None,
            )
            if section_match is not None and abs(section_match[0] - station) <= station_tolerance:
                canonical = section_match[1]
            else:
                # Use the last declaration's spelling to preserve OpenDRIVE's
                # duplicate-record precedence.
                canonical = max(cluster, key=lambda item: offsets.index(item)).get("s", ZERO)
            changed = [item.get("s", ZERO) for item in cluster if item.get("s") != canonical]
            if not changed:
                continue
            before = [item.get("s", ZERO) for item in cluster]
            for item in cluster:
                item.set("s", canonical)
            normalized_offset_stations.append(
                {
                    "road_id": road.get("id"),
                    "before": before,
                    "after": canonical,
                }
            )

    for road in tree.getroot().findall("road"):
        for section_index, section in enumerate(road.xpath("./lanes/laneSection")):
            for lane in section.xpath("./left/lane|./right/lane"):
                records = lane.findall("width") or lane.findall("border")
                if not records:
                    continue
                first_offset = float(records[0].get("sOffset", "0"))
                if first_offset <= 1.0e-9:
                    continue

                # The only valid continuation before the first record is the
                # first record itself evaluated from the section boundary.
                # VTD's affected record is constant, but retaining b/c/d also
                # makes the repair deterministic for future maps.
                initial = etree.Element(records[0].tag)
                initial.attrib.update(records[0].attrib)
                initial.set("sOffset", ZERO)
                lane.insert(lane.index(records[0]), initial)
                repaired_lane_widths.append(
                    {
                        "road_id": road.get("id"),
                        "section_index": section_index,
                        "section_s": float(section.get("s", "0")),
                        "lane_id": lane.get("id"),
                        "lane_type": lane.get("type"),
                        "record_type": records[0].tag,
                        "first_original_sOffset": first_offset,
                        "copied_coefficients": {
                            name: records[0].get(name, ZERO)
                            for name in ("a", "b", "c", "d")
                        },
                    }
                )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(
        str(output_path),
        encoding="UTF-8",
        xml_declaration=True,
        pretty_print=False,
    )
    report_path.write_text(
        json.dumps(
            {
                "input": str(input_path),
                "output": str(output_path),
                "reason": (
                    "OpenDRIVE laneOffset is implicitly zero before its first "
                    "record; an explicit record avoids CRDesigner selecting the "
                    "last record for the preceding interval."
                ),
                "affected_road_count": len(repaired),
                "affected_roads": repaired,
                "affected_lane_width_count": len(repaired_lane_widths),
                "affected_lane_widths": repaired_lane_widths,
                "normalized_offset_station_count": len(normalized_offset_stations),
                "normalized_offset_stations": normalized_offset_stations,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def main() -> None:
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("input", type=Path)
    arg_parser.add_argument("output", type=Path)
    arg_parser.add_argument("--report", type=Path, required=True)
    args = arg_parser.parse_args()
    repair(args.input, args.output, args.report)


if __name__ == "__main__":
    main()
