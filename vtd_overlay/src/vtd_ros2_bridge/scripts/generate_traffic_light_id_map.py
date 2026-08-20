#!/usr/bin/env python3
"""Build VTD/OpenDRIVE signal ID -> Autoware traffic-light group ID mappings."""

from __future__ import annotations

import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path


RED_SIGNAL_TYPE = "1000020"
AMBER_SIGNAL_TYPE = "1000008"
GREEN_SIGNAL_TYPE = "1000012"
LOCAL_SIGNAL_HEADING_TOLERANCE_RAD = 0.35
GROUP_CONTROLLER_LONGITUDINAL_TOLERANCE_M = 10.0
GROUP_CONTROLLER_LATERAL_TOLERANCE_M = 12.0
LEFT_SIGNAL_SUBTYPES = {"10", "40"}
RIGHT_SIGNAL_SUBTYPES = {"20", "50"}
STRAIGHT_SIGNAL_SUBTYPES = {"-1", "30"}


def tags(element: ET.Element) -> dict[str, str]:
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


def opendrive_lights(
    path: Path,
) -> dict[int, tuple[float, float, str, str, float]]:
    try:
        from crdesigner.map_conversion.opendrive.odr2cr.opendrive_parser.parser import (
            parse_opendrive,
        )
    except ImportError as error:
        raise RuntimeError(
            "CRDesigner is required to evaluate OpenDRIVE road geometry. "
            "Run this generator with the CRDesigner virtual environment."
        ) from error

    raw_root = ET.parse(path).getroot()
    heading_offsets = {
        int(signal.attrib["id"]): float(signal.attrib.get("hOffset", "0"))
        for signal in raw_root.findall(".//signal")
        if signal.attrib.get("dynamic") == "yes"
    }

    result: dict[int, tuple[float, float, str, str, float]] = {}
    opendrive = parse_opendrive(path)
    for road in opendrive.roads:
        road.plan_view.precalculate()
        for signal in road.signals:
            if signal.dynamic != "yes":
                continue
            position, tangent, _, _ = road.plan_view.calc(
                signal.s, compute_curvature=False
            )
            signal_id = int(signal.id)
            result[signal_id] = (
                float(position[0] + signal.t * math.cos(tangent + math.pi / 2.0)),
                float(position[1] + signal.t * math.sin(tangent + math.pi / 2.0)),
                str(signal.type),
                str(signal.subtype),
                math.atan2(
                    math.sin(tangent + heading_offsets[signal_id]),
                    math.cos(tangent + heading_offsets[signal_id]),
                ),
            )
    return result


def osm_light_ways(root: ET.Element) -> dict[int, tuple[float, float]]:
    nodes: dict[int, tuple[float, float]] = {}
    for node in root.findall("node"):
        node_tags = tags(node)
        if "local_x" in node_tags and "local_y" in node_tags:
            nodes[int(node.attrib["id"])] = (
                float(node_tags["local_x"]),
                float(node_tags["local_y"]),
            )

    result: dict[int, tuple[float, float]] = {}
    for way in root.findall("way"):
        if tags(way).get("type") != "traffic_light":
            continue
        first_node = way.find("nd")
        if first_node is None:
            continue
        position = nodes.get(int(first_node.attrib["ref"]))
        if position is not None:
            result[int(way.attrib["id"])] = position
    return result


def match_lights(
    vtd_lights: dict[int, tuple[float, float, str, str, float]],
    osm_ways: dict[int, tuple[float, float]],
    tolerance: float,
) -> dict[int, int]:
    result: dict[int, int] = {}
    unmatched: list[int] = []
    unused_way_ids = set(osm_ways)
    for vtd_id, (x, y, _signal_type, _signal_subtype, _heading) in sorted(
        vtd_lights.items()
    ):
        distance, way_id = min(
            (math.hypot(x - wx, y - wy), candidate_id)
            for candidate_id, (wx, wy) in osm_ways.items()
            if candidate_id in unused_way_ids
        )
        if distance > tolerance:
            unmatched.append(vtd_id)
        else:
            result[vtd_id] = way_id
            unused_way_ids.remove(way_id)
    if unmatched:
        raise RuntimeError(
            f"{len(unmatched)} OpenDRIVE traffic lights did not match an OSM way "
            f"within {tolerance} m; first IDs: {unmatched[:10]}"
        )
    return result


def opendrive_controllers(path: Path) -> dict[int, set[int]]:
    """Return the dynamic signal IDs controlled by each OpenDRIVE controller."""

    root = ET.parse(path).getroot()
    result: dict[int, set[int]] = {}
    for controller in root.findall("controller"):
        signal_ids = {
            int(control.attrib["signalId"])
            for control in controller.findall("control")
        }
        if signal_ids:
            result[int(controller.attrib["id"])] = signal_ids
    return result


def angle_difference(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def osm_map_geometry(
    root: ET.Element,
) -> tuple[dict[int, tuple[float, float]], dict[int, ET.Element]]:
    node_positions: dict[int, tuple[float, float]] = {}
    for node in root.findall("node"):
        node_tags = tags(node)
        if "local_x" in node_tags and "local_y" in node_tags:
            node_positions[int(node.attrib["id"])] = (
                float(node_tags["local_x"]),
                float(node_tags["local_y"]),
            )
    ways = {int(way.attrib["id"]): way for way in root.findall("way")}
    return node_positions, ways


def traffic_light_group_geometry(
    root: ET.Element, lane_headings: dict[int, float]
) -> tuple[
    dict[int, tuple[float, float]],
    dict[int, list[tuple[int, int, float]]],
    set[int],
]:
    """Return stop positions and regulated-lane headings for every light group."""

    node_positions, ways = osm_map_geometry(root)

    def way_points(way_id: int) -> list[tuple[float, float]]:
        way = ways.get(way_id)
        if way is None:
            return []
        return [
            node_positions[int(node.attrib["ref"])]
            for node in way.findall("nd")
            if int(node.attrib["ref"]) in node_positions
        ]

    group_lanes: dict[int, list[tuple[int, int, float]]] = {}
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet":
            continue
        bounds = {
            member.attrib["role"]: int(member.attrib["ref"])
            for member in relation.findall("member")
            if member.attrib.get("type") == "way"
            and member.attrib.get("role") in {"left", "right"}
        }
        if "left" not in bounds or "right" not in bounds:
            continue
        left = way_points(bounds["left"])
        right = way_points(bounds["right"])
        if len(left) < 2 or len(right) < 2:
            continue
        left_direction = (left[-1][0] - left[0][0], left[-1][1] - left[0][1])
        right_direction = (
            right[-1][0] - right[0][0],
            right[-1][1] - right[0][1],
        )
        if (
            left_direction[0] * right_direction[0]
            + left_direction[1] * right_direction[1]
            < 0.0
        ):
            right.reverse()
        previous = (
            (left[-2][0] + right[-2][0]) / 2.0,
            (left[-2][1] + right[-2][1]) / 2.0,
        )
        last = (
            (left[-1][0] + right[-1][0]) / 2.0,
            (left[-1][1] + right[-1][1]) / 2.0,
        )
        source_id = int(relation_tags.get("source_commonroad_id", "-1"))
        heading = lane_headings.get(
            source_id,
            math.atan2(last[1] - previous[1], last[0] - previous[0]),
        )
        for member in relation.findall("member"):
            if (
                member.attrib.get("type") == "relation"
                and member.attrib.get("role") == "regulatory_element"
            ):
                group_lanes.setdefault(int(member.attrib["ref"]), []).append(
                    (int(relation.attrib["id"]), source_id, heading)
                )

    stop_positions: dict[int, tuple[float, float]] = {}
    traffic_light_group_ids: set[int] = set()
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if (
            relation_tags.get("type") != "regulatory_element"
            or relation_tags.get("subtype") != "traffic_light"
        ):
            continue
        group_id = int(relation.attrib["id"])
        traffic_light_group_ids.add(group_id)
        ref_lines = [
            int(member.attrib["ref"])
            for member in relation.findall("member")
            if member.attrib.get("type") == "way"
            and member.attrib.get("role") == "ref_line"
        ]
        if not ref_lines:
            continue
        stop_line = way_points(ref_lines[0])
        if stop_line:
            stop_positions[group_id] = (
                sum(point[0] for point in stop_line) / len(stop_line),
                sum(point[1] for point in stop_line) / len(stop_line),
            )
    return stop_positions, group_lanes, traffic_light_group_ids


def commonroad_lane_context(
    path: Path,
) -> tuple[dict[int, set[str]], dict[int, float]]:
    """Return intersection movements and travel headings from CommonRoad."""

    try:
        from commonroad.common.file_reader import CommonRoadFileReader
    except ImportError as error:
        raise RuntimeError(
            "CommonRoad is required when --commonroad is specified."
        ) from error

    scenario, _ = CommonRoadFileReader(path).open()
    maneuvers: dict[int, set[str]] = {}
    headings: dict[int, float] = {}
    for lanelet in scenario.lanelet_network.lanelets:
        center_vertices = lanelet.center_vertices
        if len(center_vertices) < 2:
            continue
        for index in range(len(center_vertices) - 1, 0, -1):
            dx = float(center_vertices[index][0] - center_vertices[index - 1][0])
            dy = float(center_vertices[index][1] - center_vertices[index - 1][1])
            if math.hypot(dx, dy) > 1.0e-6:
                headings[int(lanelet.lanelet_id)] = math.atan2(dy, dx)
                break
    for intersection in scenario.lanelet_network.intersections:
        for incoming in intersection.incomings:
            directions: set[str] = set()
            if incoming.successors_left:
                directions.add("left")
            if incoming.successors_right:
                directions.add("right")
            if incoming.successors_straight:
                directions.add("straight")
            for lanelet_id in incoming.incoming_lanelets:
                maneuvers.setdefault(int(lanelet_id), set()).update(directions)
    return maneuvers, headings


def preferred_controller_shape(
    directions: set[str], signal_ids: set[int], vtd_lights: dict[int, tuple]
) -> int:
    green_subtypes = {
        vtd_lights[signal_id][3]
        for signal_id in signal_ids
        if signal_id in vtd_lights
        and vtd_lights[signal_id][2] == GREEN_SIGNAL_TYPE
    }
    if "left" in directions:
        return 0 if green_subtypes & LEFT_SIGNAL_SUBTYPES else 1
    if "straight" in directions:
        return 0 if green_subtypes & STRAIGHT_SIGNAL_SUBTYPES else 1
    if "right" in directions:
        return 0 if green_subtypes & RIGHT_SIGNAL_SUBTYPES else 1
    return 0


def match_groups_to_controllers(
    root: ET.Element,
    vtd_lights: dict[int, tuple[float, float, str, str, float]],
    controllers: dict[int, set[int]],
    maneuvers: dict[int, set[str]],
    lane_headings: dict[int, float],
) -> tuple[dict[int, set[int]], dict[int, int], set[int], Counter[str]]:
    """Match each map group to one nearby, aligned VTD signal controller."""

    stop_positions, group_lanes, traffic_light_group_ids = traffic_light_group_geometry(
        root, lane_headings
    )
    required_types = {RED_SIGNAL_TYPE, AMBER_SIGNAL_TYPE, GREEN_SIGNAL_TYPE}
    valid_controllers = {
        controller_id: {
            signal_id
            for signal_id in signal_ids
            if signal_id in vtd_lights
        }
        for controller_id, signal_ids in controllers.items()
        if required_types.issubset(
            {
                vtd_lights[signal_id][2]
                for signal_id in signal_ids
                if signal_id in vtd_lights
            }
        )
    }

    matches: dict[int, set[int]] = {}
    matched_controllers: dict[int, int] = {}
    right_only_source_ids = {
        source_id for source_id, directions in maneuvers.items() if directions == {"right"}
    }
    reason_counts: Counter[str] = Counter()

    for group_id in sorted(traffic_light_group_ids):
        stop_position = stop_positions.get(group_id)
        lane_contexts = group_lanes.get(group_id, [])
        if stop_position is None or not lane_contexts:
            reason_counts["missing_stop_line_or_lane"] += 1
            continue

        candidates: list[tuple[float, float, float, float, int, int, set[int]]] = []
        has_non_right_lane = False
        for _lane_id, source_id, lane_heading in lane_contexts:
            directions = maneuvers.get(source_id, set())
            if directions == {"right"}:
                continue
            has_non_right_lane = True
            for controller_id, signal_ids in valid_controllers.items():
                signals = [vtd_lights[signal_id] for signal_id in signal_ids]
                heading_error = max(
                    angle_difference(signal[4], lane_heading) for signal in signals
                )
                longitudinal = max(
                    abs(
                        (signal[0] - stop_position[0]) * math.cos(lane_heading)
                        + (signal[1] - stop_position[1]) * math.sin(lane_heading)
                    )
                    for signal in signals
                )
                lateral = max(
                    abs(
                        -(signal[0] - stop_position[0]) * math.sin(lane_heading)
                        + (signal[1] - stop_position[1]) * math.cos(lane_heading)
                    )
                    for signal in signals
                )
                if (
                    heading_error > LOCAL_SIGNAL_HEADING_TOLERANCE_RAD
                    or longitudinal > GROUP_CONTROLLER_LONGITUDINAL_TOLERANCE_M
                    or lateral > GROUP_CONTROLLER_LATERAL_TOLERANCE_M
                ):
                    continue
                center_x = sum(signal[0] for signal in signals) / len(signals)
                center_y = sum(signal[1] for signal in signals) / len(signals)
                center_distance = math.hypot(
                    center_x - stop_position[0], center_y - stop_position[1]
                )
                shape_penalty = preferred_controller_shape(
                    directions, signal_ids, vtd_lights
                )
                candidates.append(
                    (
                        float(shape_penalty),
                        center_distance,
                        longitudinal,
                        lateral,
                        controller_id,
                        source_id,
                        signal_ids,
                    )
                )

        if not has_non_right_lane:
            reason_counts["right_turn_only"] += 1
            continue
        if not candidates:
            reason_counts["no_local_aligned_controller"] += 1
            continue

        candidates.sort(key=lambda item: item[:-1])
        selected = candidates[0]
        matches[group_id] = selected[-1]
        matched_controllers[group_id] = selected[4]
        reason_counts["matched"] += 1

    return matches, matched_controllers, right_only_source_ids, reason_counts


def clean_osm_traffic_light_relations(
    root: ET.Element,
    group_signal_way_ids: dict[int, set[int]],
    right_only_source_ids: set[int],
) -> dict[str, int]:
    """Remove phantom groups and traffic-light rules from right-turn-only lanes."""

    traffic_light_group_ids = {
        int(relation.attrib["id"])
        for relation in root.findall("relation")
        if tags(relation).get("type") == "regulatory_element"
        and tags(relation).get("subtype") == "traffic_light"
    }
    matched_group_ids = set(group_signal_way_ids)
    removed_lane_members = 0
    removed_right_turn_members = 0
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet":
            continue
        source_id = int(relation_tags.get("source_commonroad_id", "-1"))
        for member in list(relation.findall("member")):
            if (
                member.attrib.get("type") != "relation"
                or member.attrib.get("role") != "regulatory_element"
            ):
                continue
            group_id = int(member.attrib["ref"])
            if group_id not in traffic_light_group_ids:
                continue
            if source_id in right_only_source_ids:
                relation.remove(member)
                removed_lane_members += 1
                removed_right_turn_members += 1
            elif group_id not in matched_group_ids:
                relation.remove(member)
                removed_lane_members += 1

    rewritten_groups = 0
    rewritten_refers_members = 0
    for relation in root.findall("relation"):
        group_id = int(relation.attrib["id"])
        expected_way_ids = group_signal_way_ids.get(group_id)
        if expected_way_ids is None:
            continue
        refers_members = [
            member
            for member in relation.findall("member")
            if member.attrib.get("type") == "way"
            and member.attrib.get("role") == "refers"
        ]
        actual_way_ids = {int(member.attrib["ref"]) for member in refers_members}
        if actual_way_ids == expected_way_ids:
            continue
        for member in refers_members:
            relation.remove(member)
        insertion_index = next(
            (
                index
                for index, child in enumerate(list(relation))
                if child.tag == "tag"
            ),
            len(relation),
        )
        for way_id in sorted(expected_way_ids):
            relation.insert(
                insertion_index,
                ET.Element(
                    "member",
                    {"type": "way", "ref": str(way_id), "role": "refers"},
                ),
            )
            insertion_index += 1
        rewritten_groups += 1
        rewritten_refers_members += len(expected_way_ids)

    referenced_group_ids = {
        int(member.attrib["ref"])
        for relation in root.findall("relation")
        if tags(relation).get("type") == "lanelet"
        for member in relation.findall("member")
        if member.attrib.get("type") == "relation"
        and member.attrib.get("role") == "regulatory_element"
    }
    removed_groups = 0
    for relation in list(root.findall("relation")):
        relation_tags = tags(relation)
        if (
            relation_tags.get("type") == "regulatory_element"
            and relation_tags.get("subtype") == "traffic_light"
            and int(relation.attrib["id"]) not in referenced_group_ids
        ):
            root.remove(relation)
            removed_groups += 1

    return {
        "removed_lane_regulatory_members": removed_lane_members,
        "removed_right_turn_only_members": removed_right_turn_members,
        "removed_unreferenced_traffic_light_groups": removed_groups,
        "rewritten_traffic_light_groups": rewritten_groups,
        "rewritten_refers_members": rewritten_refers_members,
        "remaining_traffic_light_groups": len(
            traffic_light_group_ids & referenced_group_ids
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("opendrive", type=Path)
    parser.add_argument("osm", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--tolerance", type=float, default=0.02)
    parser.add_argument(
        "--commonroad",
        type=Path,
        help="CommonRoad source used to identify right-turn-only incoming lanes.",
    )
    parser.add_argument(
        "--cleaned-osm-output",
        type=Path,
        help="Write a map with phantom and right-turn-only light rules removed.",
    )
    parser.add_argument(
        "--audit-report",
        type=Path,
        help="Write the full-map traffic-light cleanup statistics as JSON.",
    )
    args = parser.parse_args()

    if args.cleaned_osm_output is not None and args.commonroad is None:
        parser.error("--cleaned-osm-output requires --commonroad")

    osm_tree = ET.parse(args.osm)
    osm_root = osm_tree.getroot()
    vtd_lights = opendrive_lights(args.opendrive)
    controllers = opendrive_controllers(args.opendrive)
    # Validate that the converted map still contains every physical VTD lamp,
    # but do not use converter-provided ``refers`` members for group matching.
    matched_light_ways = match_lights(
        vtd_lights, osm_light_ways(osm_root), args.tolerance
    )
    maneuvers, lane_headings = (
        commonroad_lane_context(args.commonroad)
        if args.commonroad is not None
        else ({}, {})
    )
    group_signals, matched_controllers, right_only_source_ids, reason_counts = (
        match_groups_to_controllers(
            osm_root, vtd_lights, controllers, maneuvers, lane_headings
        )
    )
    pairs = {
        (vtd_id, group_id, vtd_lights[vtd_id][2], vtd_lights[vtd_id][3])
        for group_id, vtd_ids in group_signals.items()
        for vtd_id in vtd_ids
    }
    sorted_pairs = sorted(pairs)

    cleanup_stats: dict[str, int] = {}
    if args.cleaned_osm_output is not None:
        group_signal_way_ids = {
            group_id: {
                matched_light_ways[vtd_id]
                for vtd_id in vtd_ids
                if vtd_id in matched_light_ways
            }
            for group_id, vtd_ids in group_signals.items()
        }
        cleanup_stats = clean_osm_traffic_light_relations(
            osm_root, group_signal_way_ids, right_only_source_ids
        )
        args.cleaned_osm_output.parent.mkdir(parents=True, exist_ok=True)
        osm_tree.write(
            args.cleaned_osm_output,
            encoding="UTF-8",
            xml_declaration=True,
            short_empty_elements=True,
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as output:
        output.write(
            "# Generated by local stop-line/heading/controller matching; "
            "converter refers links are ignored.\n"
        )
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(
            ("vtd_id", "autoware_group_id", "signal_type", "signal_subtype")
        )
        writer.writerows(sorted_pairs)

    audit = {
        "opendrive_dynamic_signals": len(vtd_lights),
        "opendrive_controllers": len(controllers),
        "osm_matched_signal_ways": len(matched_light_ways),
        "matched_traffic_light_groups": len(group_signals),
        "matched_controllers": len(set(matched_controllers.values())),
        "mapping_pairs": len(sorted_pairs),
        "right_turn_only_source_lanelets": len(right_only_source_ids),
        "matching_results": dict(sorted(reason_counts.items())),
        "cleanup": cleanup_stats,
    }
    if args.audit_report is not None:
        args.audit_report.parent.mkdir(parents=True, exist_ok=True)
        args.audit_report.write_text(
            json.dumps(audit, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    mapped_vtd_ids = len({vtd_id for vtd_id, *_ in sorted_pairs})
    print(
        f"wrote {len(sorted_pairs)} pairs for {mapped_vtd_ids} VTD signals and "
        f"{len(group_signals)} local groups to {args.output}; "
        f"cleanup={cleanup_stats or 'not requested'}"
    )


if __name__ == "__main__":
    main()
