#!/usr/bin/env python3
"""Convert the repaired VTD OpenDRIVE map to CommonRoad reproducibly."""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Tag

from crdesigner.common.config.general_config import general_config
from crdesigner.common.config.opendrive_config import open_drive_config
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_commonroad
import crdesigner.map_conversion.opendrive.odr2cr.opendrive_conversion.network as network_module


def convert(input_path: Path, output_path: Path, report_path: Path) -> None:
    logging.disable(logging.WARNING)
    skipped_crosswalk_roads: list[int | str] = []
    original_get_crosswalks = network_module.get_crosswalks

    def safe_get_crosswalks(road):
        try:
            return original_get_crosswalks(road)
        except NotImplementedError:
            # Shapely 2 rejects direct coordinate access on a multipart object
            # produced by CRDesigner's crosswalk extractor.  Keep all other
            # road objects and record the affected road explicitly.
            skipped_crosswalk_roads.append(road.id)
            return []

    network_module.get_crosswalks = safe_get_crosswalks

    general_config.author = "OpenAI"
    general_config.affiliation = "VTD to Autoware map conversion"
    general_config.source = "VTD OpenDRIVE (lane topology repaired)"

    scenario = opendrive_to_commonroad(
        input_path, general_conf=general_config, odr_conf=open_drive_config
    )
    writer = CommonRoadFileWriter(
        scenario=scenario,
        planning_problem_set=PlanningProblemSet(),
        author=general_config.author,
        affiliation=general_config.affiliation,
        source=general_config.source,
        tags={Tag.URBAN},
        location=scenario.location,
        decimal_precision=10,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer.write_to_file(
        str(output_path),
        overwrite_existing_file=OverwriteExistingFile.ALWAYS,
        check_validity=False,
    )
    report_path.write_text(
        json.dumps(
            {
                "input": str(input_path),
                "output": str(output_path),
                "lanelets": len(scenario.lanelet_network.lanelets),
                "traffic_signs": len(scenario.lanelet_network.traffic_signs),
                "traffic_lights": len(scenario.lanelet_network.traffic_lights),
                "intersections": len(scenario.lanelet_network.intersections),
                "crosswalk_extraction_skipped_roads": skipped_crosswalk_roads,
                "filter_types": list(open_drive_config.filter_types),
                "error_tolerance": open_drive_config.error_tolerance,
                "min_delta_s": open_drive_config.min_delta_s,
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
    convert(args.input, args.output, args.report)


if __name__ == "__main__":
    main()
