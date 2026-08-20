#!/usr/bin/env python3
"""Export repaired CommonRoad XML as a cleaned Autoware Lanelet2 map."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path

from lxml import etree
import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from crdesigner.common.config.lanelet2_config import lanelet2_config
from crdesigner.map_conversion.map_conversion_interface import commonroad_to_lanelet
import crdesigner.map_conversion.lanelet2.cr2lanelet as cr2lanelet_module
from shapely.geometry import LineString, Polygon


ENDPOINT_SHARE_TOLERANCE_M = 0.75
ADJACENCY_SHARE_TOLERANCE_M = 0.15
# These two pairs are geometrically close, but sharing either candidate way
# self-intersects one polygon after the longitudinal endpoint snap.  Keeping
# separate boundaries is the only topology-safe representation.
UNSAFE_ADJACENCY_PAIRS = {(1627, 1631), (1664, 1667)}


def vertices_are_equal(vertices1, vertices2, tolerance: float) -> bool:
    """Correct CRDesigner's signed-maximum boundary comparison.

    The installed implementation applies ``abs`` after ``max``.  A boundary
    that differs by -3 m at most samples can consequently compare equal when
    just one sample is near zero, causing a neighboring lane's way to be
    reused.  The maximum absolute component is the intended test.
    """
    if len(vertices1) != len(vertices2):
        return False
    difference = np.asarray(vertices1) - np.asarray(vertices2)
    # A repaired taper can legitimately differ from its former adjacent border
    # by only a few nanometres at the collapsed endpoint.  Reusing that way
    # reintroduces a self-intersection, so only share boundaries that agree to
    # writing precision.
    effective_tolerance = min(tolerance, 1.0e-9)
    return bool(np.max(np.abs(difference)) < effective_tolerance)


def _endpoint_distance(first: np.ndarray, second: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(first)[:2] - np.asarray(second)[:2]))


def _best_endpoint_pair(candidates):
    candidates = [candidate for candidate in candidates if candidate[1][0] and candidate[1][1]]
    if not candidates:
        return None, None
    score, nodes = min(candidates, key=lambda candidate: candidate[0])
    if score > ENDPOINT_SHARE_TOLERANCE_M:
        return None, None
    return nodes


def shared_first_nodes_with_geometry_check(self, lanelet):
    """Reuse predecessor nodes only when both lane boundaries actually meet.

    CRDesigner's exporter reuses the first already-converted predecessor or
    sibling endpoint without checking its coordinates.  At a lane split this
    can replace a full-width boundary with the zero-width tip of the new lane,
    shifting one side by an entire lane width.
    """
    candidates = []
    for predecessor_id in lanelet.predecessor or []:
        predecessor = self.lanelet_network.find_lanelet_by_id(predecessor_id)
        nodes = self.last_nodes.get(predecessor_id, (None, None))
        score = max(
            _endpoint_distance(lanelet.left_vertices[0], predecessor.left_vertices[-1]),
            _endpoint_distance(lanelet.right_vertices[0], predecessor.right_vertices[-1]),
        )
        candidates.append((score, nodes))
    for predecessor_id in lanelet.predecessor or []:
        predecessor = self.lanelet_network.find_lanelet_by_id(predecessor_id)
        for sibling_id in predecessor.successor or []:
            if sibling_id == lanelet.lanelet_id:
                continue
            sibling = self.lanelet_network.find_lanelet_by_id(sibling_id)
            nodes = self.first_nodes.get(sibling_id, (None, None))
            score = max(
                _endpoint_distance(lanelet.left_vertices[0], sibling.left_vertices[0]),
                _endpoint_distance(lanelet.right_vertices[0], sibling.right_vertices[0]),
            )
            candidates.append((score, nodes))
    return _best_endpoint_pair(candidates)


def shared_last_nodes_with_geometry_check(self, lanelet):
    """Reuse successor nodes only when both lane boundaries actually meet."""
    candidates = []
    for successor_id in lanelet.successor or []:
        successor = self.lanelet_network.find_lanelet_by_id(successor_id)
        nodes = self.first_nodes.get(successor_id, (None, None))
        score = max(
            _endpoint_distance(lanelet.left_vertices[-1], successor.left_vertices[0]),
            _endpoint_distance(lanelet.right_vertices[-1], successor.right_vertices[0]),
        )
        candidates.append((score, nodes))
    for successor_id in lanelet.successor or []:
        successor = self.lanelet_network.find_lanelet_by_id(successor_id)
        for sibling_id in successor.predecessor or []:
            if sibling_id == lanelet.lanelet_id:
                continue
            sibling = self.lanelet_network.find_lanelet_by_id(sibling_id)
            nodes = self.last_nodes.get(sibling_id, (None, None))
            score = max(
                _endpoint_distance(lanelet.left_vertices[-1], sibling.left_vertices[-1]),
                _endpoint_distance(lanelet.right_vertices[-1], sibling.right_vertices[-1]),
            )
            candidates.append((score, nodes))
    return _best_endpoint_pair(candidates)


def share_geometry_compatible_adjacencies(root, commonroad_path: Path):
    """Share a Lanelet2 boundary only for geometrically coincident neighbors."""
    scenario, _ = CommonRoadFileReader(commonroad_path).open()
    commonroad_lanelets = list(scenario.lanelet_network.lanelets)
    lanelets_by_id = {lanelet.lanelet_id: lanelet for lanelet in commonroad_lanelets}
    relations = []
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("type") == "lanelet":
            relations.append(relation)
    if len(relations) != len(commonroad_lanelets):
        raise RuntimeError("Cannot align lanelets while checking adjacency")
    relations_by_id = dict(zip((lanelet.lanelet_id for lanelet in commonroad_lanelets), relations))
    ways = {way.get("id"): way for way in root.findall("way")}
    node_coordinates = {}
    for node in root.findall("node"):
        tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
        node_coordinates[node.get("id")] = (
            float(tags.get("local_x", node.get("lon", "0"))),
            float(tags.get("local_y", node.get("lat", "0"))),
        )

    def relation_polygon_is_valid(lanelet, relation, override):
        members = {
            member.get("role"): override.get(member.get("role"), member.get("ref"))
            for member in relation.findall("member")
            if member.get("type") == "way"
        }
        boundaries = {}
        for side in ("left", "right"):
            references = [nd.get("ref") for nd in ways[members[side]].findall("nd")]
            points = [node_coordinates[reference] for reference in references]
            vertices = getattr(lanelet, f"{side}_vertices")
            direct = math.dist(points[0], vertices[0][:2]) + math.dist(
                points[-1], vertices[-1][:2]
            )
            reverse = math.dist(points[-1], vertices[0][:2]) + math.dist(
                points[0], vertices[-1][:2]
            )
            boundaries[side] = points if direct <= reverse else points[::-1]
        ring = boundaries["left"] + boundaries["right"][::-1]
        polygon = Polygon(ring)
        float32_polygon = Polygon(np.asarray(ring, dtype=np.float32))
        return (
            polygon.is_valid
            and polygon.area > 1.0e-8
            and float32_polygon.is_valid
            and float32_polygon.area > 1.0e-8
        )

    checked = set()
    shared = []
    rejected = []
    compatible_but_unsafe = []
    for lanelet in commonroad_lanelets:
        for side in ("left", "right"):
            adjacent_id = getattr(lanelet, f"adj_{side}")
            same_direction = getattr(lanelet, f"adj_{side}_same_direction")
            if adjacent_id is None:
                continue
            pair = tuple(sorted((lanelet.lanelet_id, adjacent_id)))
            if pair in checked:
                continue
            checked.add(pair)
            adjacent = lanelets_by_id[adjacent_id]
            if side == "right":
                adjacent_side = "left" if same_direction else "right"
            else:
                adjacent_side = "right" if same_direction else "left"
            first_vertices = getattr(lanelet, f"{side}_vertices")[:, :2]
            second_vertices = getattr(adjacent, f"{adjacent_side}_vertices")[:, :2]
            distance = float(
                LineString(first_vertices).hausdorff_distance(LineString(second_vertices))
            )
            first_relation = relations_by_id[lanelet.lanelet_id]
            second_relation = relations_by_id[adjacent_id]
            first_member = next(
                member
                for member in first_relation.findall("member")
                if member.get("type") == "way" and member.get("role") == side
            )
            second_member = next(
                member
                for member in second_relation.findall("member")
                if member.get("type") == "way" and member.get("role") == adjacent_side
            )
            item = {
                "first_commonroad_id": str(lanelet.lanelet_id),
                "first_side": side,
                "second_commonroad_id": str(adjacent_id),
                "second_side": adjacent_side,
                "same_direction": bool(same_direction),
                "boundary_hausdorff_m": distance,
            }
            if distance <= ADJACENCY_SHARE_TOLERANCE_M:
                if pair in UNSAFE_ADJACENCY_PAIRS:
                    item["reason"] = (
                        "sharing this boundary self-intersects a lanelet after endpoint snapping"
                    )
                    compatible_but_unsafe.append(item)
                    continue
                first_way, second_way = first_member.get("ref"), second_member.get("ref")
                if first_way != second_way:
                    candidates = sorted(
                        (first_way, second_way),
                        key=lambda way_id: (-len(ways[way_id].findall("nd")), int(way_id)),
                    )
                    canonical = next(
                        (
                            way_id
                            for way_id in candidates
                            if relation_polygon_is_valid(
                                lanelet, first_relation, {side: way_id}
                            )
                            and relation_polygon_is_valid(
                                adjacent, second_relation, {adjacent_side: way_id}
                            )
                        ),
                        None,
                    )
                    if canonical is None:
                        item["reason"] = (
                            "sharing either candidate boundary would invalidate a lanelet polygon"
                        )
                        compatible_but_unsafe.append(item)
                        continue
                    first_member.set("ref", canonical)
                    second_member.set("ref", canonical)
                    item["shared_way"] = canonical
                    item["replaced_way"] = second_way if canonical == first_way else first_way
                else:
                    item["shared_way"] = first_way
                shared.append(item)
            else:
                item["reason"] = "declared adjacency is not geometrically coincident"
                rejected.append(item)
    return {
        "declared_adjacency_pairs": len(checked),
        "geometry_compatible_pairs": len(shared),
        "noncoincident_pairs_not_forced": len(rejected),
        "compatible_but_unsafe_pairs_not_forced": len(compatible_but_unsafe),
        "newly_shared_pairs": sum("replaced_way" in item for item in shared),
        "maximum_shared_boundary_hausdorff_m": max(
            (item["boundary_hausdorff_m"] for item in shared), default=None
        ),
        "largest_rejected_pairs": sorted(
            rejected, key=lambda item: item["boundary_hausdorff_m"], reverse=True
        )[:20],
    }


def snap_successor_endpoints(root, commonroad_path: Path):
    """Make all geometrically compatible CommonRoad successors routable.

    Adjacency way reuse can override the exporter's predecessor node reuse and
    leave millimetre-scale breaks.  Union only the oriented endpoint node IDs;
    full-lane-width taper tips are deliberately left untouched.
    """
    scenario, _ = CommonRoadFileReader(commonroad_path).open()
    commonroad_lanelets = list(scenario.lanelet_network.lanelets)
    lanelets_by_id = {lanelet.lanelet_id: lanelet for lanelet in commonroad_lanelets}

    lanelet_relations = []
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("type") == "lanelet":
            lanelet_relations.append(relation)
    if len(lanelet_relations) != len(commonroad_lanelets):
        raise RuntimeError(
            "Cannot align CommonRoad and OSM lanelets: "
            f"{len(commonroad_lanelets)} != {len(lanelet_relations)}"
        )

    nodes = {}
    for node in root.findall("node"):
        tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
        nodes[node.get("id")] = (
            float(tags.get("local_x", node.get("lon", "0"))),
            float(tags.get("local_y", node.get("lat", "0"))),
        )
    ways = {way.get("id"): way for way in root.findall("way")}

    def oriented_endpoints(way_id, vertices):
        references = [nd.get("ref") for nd in ways[way_id].findall("nd")]
        if len(references) < 2:
            raise RuntimeError(f"way {way_id} has fewer than two nodes")
        first, last = references[0], references[-1]
        direct = math.dist(nodes[first], vertices[0][:2]) + math.dist(
            nodes[last], vertices[-1][:2]
        )
        reverse = math.dist(nodes[last], vertices[0][:2]) + math.dist(
            nodes[first], vertices[-1][:2]
        )
        return (first, last) if direct <= reverse else (last, first)

    endpoints = {}
    relation_ids = {}
    for lanelet, relation in zip(commonroad_lanelets, lanelet_relations):
        members = {
            member.get("role"): member.get("ref")
            for member in relation.findall("member")
            if member.get("type") == "way"
        }
        left_start, left_end = oriented_endpoints(members["left"], lanelet.left_vertices)
        right_start, right_end = oriented_endpoints(members["right"], lanelet.right_vertices)
        endpoints[lanelet.lanelet_id] = {
            "left_start": left_start,
            "left_end": left_end,
            "right_start": right_start,
            "right_end": right_end,
        }
        relation_ids[lanelet.lanelet_id] = relation.get("id")
        if not any(
            tag.get("k") == "source_commonroad_id" for tag in relation.findall("tag")
        ):
            etree.SubElement(
                relation, "tag", k="source_commonroad_id", v=str(lanelet.lanelet_id)
            )

    parent = {node_id: node_id for node_id in nodes}

    def find(node_id):
        while parent[node_id] != node_id:
            parent[node_id] = parent[parent[node_id]]
            node_id = parent[node_id]
        return node_id

    def union(first, second):
        first_root, second_root = find(first), find(second)
        if first_root != second_root:
            parent[second_root] = first_root

    accepted = []
    rejected = []
    for source in commonroad_lanelets:
        for target_id in source.successor:
            target = lanelets_by_id[target_id]
            left_gap = _endpoint_distance(source.left_vertices[-1], target.left_vertices[0])
            right_gap = _endpoint_distance(source.right_vertices[-1], target.right_vertices[0])
            item = {
                "source_commonroad_id": str(source.lanelet_id),
                "target_commonroad_id": str(target_id),
                "source_osm_id": relation_ids[source.lanelet_id],
                "target_osm_id": relation_ids[target_id],
                "left_gap_m": left_gap,
                "right_gap_m": right_gap,
            }
            if max(left_gap, right_gap) <= ENDPOINT_SHARE_TOLERANCE_M:
                source_nodes, target_nodes = endpoints[source.lanelet_id], endpoints[target_id]
                union(source_nodes["left_end"], target_nodes["left_start"])
                union(source_nodes["right_end"], target_nodes["right_start"])
                accepted.append(item)
            else:
                item["reason"] = "taper tip differs by approximately one lane width"
                rejected.append(item)

    usage = Counter(
        nd.get("ref") for way in root.findall("way") for nd in way.findall("nd")
    )
    groups = {}
    for node_id in nodes:
        groups.setdefault(find(node_id), []).append(node_id)
    replacement = {}
    for members in groups.values():
        if len(members) < 2:
            continue
        canonical = min(
            members,
            key=lambda node_id: (-usage[node_id], int(node_id)),
        )
        for node_id in members:
            replacement[node_id] = canonical
    for way in root.findall("way"):
        for nd in way.findall("nd"):
            nd.set("ref", replacement.get(nd.get("ref"), nd.get("ref")))

    return {
        "commonroad_successor_edges": sum(len(lanelet.successor) for lanelet in commonroad_lanelets),
        "snapped_successor_edges": len(accepted),
        "preserved_taper_edges": rejected,
        "endpoint_nodes_unified": sum(
            len(set(members)) - 1 for members in groups.values() if len(members) > 1
        ),
    }


def add_turn_direction_tags(root, commonroad_path: Path):
    """Preserve CommonRoad intersection maneuvers in Lanelet2 relations.

    CRDesigner's Lanelet2 exporter omits the CommonRoad intersection model.
    Autoware therefore cannot distinguish an intersection turn from an ordinary
    routed lanelet.  Tag each intersection successor lanelet with the direction
    already encoded in the CommonRoad network.
    """
    scenario, _ = CommonRoadFileReader(commonroad_path).open()
    directions: dict[int, str] = {}
    reference_counts: Counter[str] = Counter()
    for intersection in scenario.lanelet_network.intersections:
        for incoming in intersection.incomings:
            successors = (
                ("left", incoming.successors_left),
                ("right", incoming.successors_right),
                ("straight", incoming.successors_straight),
            )
            for direction, lanelet_ids in successors:
                for lanelet_id in lanelet_ids:
                    previous = directions.get(lanelet_id)
                    if previous is not None and previous != direction:
                        raise RuntimeError(
                            f"Conflicting turn directions for CommonRoad lanelet {lanelet_id}: "
                            f"{previous} and {direction}"
                        )
                    directions[lanelet_id] = direction
                    reference_counts[direction] += 1

    relation_by_commonroad_id = {}
    removed_existing = 0
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        source_id = tags.get("source_commonroad_id")
        if source_id is not None:
            relation_by_commonroad_id[int(source_id)] = relation
        for tag in list(relation.findall("tag")):
            if tag.get("k") == "turn_direction":
                relation.remove(tag)
                removed_existing += 1

    missing = sorted(set(directions) - set(relation_by_commonroad_id))
    if missing:
        raise RuntimeError(
            "Cannot map CommonRoad turn-direction lanelets to OSM relations: "
            + ", ".join(map(str, missing[:20]))
        )

    unique_counts: Counter[str] = Counter()
    for lanelet_id, direction in directions.items():
        etree.SubElement(
            relation_by_commonroad_id[lanelet_id],
            "tag",
            k="turn_direction",
            v=direction,
        )
        unique_counts[direction] += 1

    return {
        "tagged_lanelets": len(directions),
        "unique_counts": dict(sorted(unique_counts.items())),
        "source_reference_counts": dict(sorted(reference_counts.items())),
        "removed_existing_tags": removed_existing,
    }


def export(input_path: Path, output_path: Path, report_path: Path) -> None:
    cr2lanelet_module._vertices_are_equal = vertices_are_equal
    cr2lanelet_module.CR2LaneletConverter._get_shared_first_nodes_from_other_lanelets = (
        shared_first_nodes_with_geometry_check
    )
    cr2lanelet_module.CR2LaneletConverter._get_shared_last_nodes_from_other_lanelets = (
        shared_last_nodes_with_geometry_check
    )
    lanelet2_config.autoware = True
    lanelet2_config.use_local_coordinates = True
    commonroad_to_lanelet(input_path, str(output_path), config=lanelet2_config)

    tree = etree.parse(str(output_path), etree.XMLParser(huge_tree=True))
    root = tree.getroot()
    meta = root.find("MetaInfo")
    if meta is None:
        meta = etree.Element("MetaInfo", format_version="1.0", map_version="1.0")
        root.insert(0, meta)
    else:
        meta.set("format_version", "1.0")
        meta.set("map_version", "1.0")

    adjacency_cleanup = share_geometry_compatible_adjacencies(root, input_path)
    topology_cleanup = snap_successor_endpoints(root, input_path)
    turn_direction_cleanup = add_turn_direction_tags(root, input_path)

    node_coordinates: dict[str, tuple[float, float]] = {}
    for node in root.findall("node"):
        tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            node_coordinates[node.get("id")] = (
                float(tags["local_x"]),
                float(tags["local_y"]),
            )
        else:
            node_coordinates[node.get("id")] = (
                float(node.get("lon", "0")),
                float(node.get("lat", "0")),
            )

    removed_consecutive_nodes = 0
    degenerate_way_ids: list[str] = []
    degenerate_way_tags: dict[str, dict[str, str]] = {}
    for way in list(root.findall("way")):
        kept: list[etree._Element] = []
        previous_ref: str | None = None
        previous_coordinate: tuple[float, float] | None = None
        for nd in way.findall("nd"):
            ref = nd.get("ref")
            coordinate = node_coordinates.get(ref)
            duplicate = ref == previous_ref
            if previous_coordinate is not None and coordinate is not None:
                duplicate = duplicate or math.dist(previous_coordinate, coordinate) <= 1.0e-9
            if duplicate:
                way.remove(nd)
                removed_consecutive_nodes += 1
                continue
            kept.append(nd)
            previous_ref = ref
            previous_coordinate = coordinate

        distinct = {nd.get("ref") for nd in kept}
        if len(kept) < 2 or len(distinct) < 2:
            way_id = way.get("id")
            degenerate_way_ids.append(way_id)
            degenerate_way_tags[way_id] = {
                tag.get("k", ""): tag.get("v", "") for tag in way.findall("tag")
            }
            root.remove(way)

    removed_relation_members = 0
    degenerate_set = set(degenerate_way_ids)
    for relation in root.findall("relation"):
        for member in list(relation.findall("member")):
            if member.get("type") == "way" and member.get("ref") in degenerate_set:
                relation.remove(member)
                removed_relation_members += 1

    # Deduplication can discard the just-unified endpoint when the preceding
    # sample has the same coordinate under another node ID.  A second pass on
    # the cleaned ways makes the node identity (not only the coordinate) exact.
    topology_cleanup["post_deduplication_pass"] = snap_successor_endpoints(
        root, input_path
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
                "autoware": True,
                "use_local_coordinates": True,
                "fixed_signed_boundary_comparison": True,
                "geometry_checked_endpoint_sharing": True,
                "endpoint_share_tolerance_m": ENDPOINT_SHARE_TOLERANCE_M,
                "topology_cleanup": topology_cleanup,
                "adjacency_cleanup": adjacency_cleanup,
                "turn_direction_cleanup": turn_direction_cleanup,
                "removed_consecutive_way_nodes": removed_consecutive_nodes,
                "removed_degenerate_ways": degenerate_way_ids,
                "degenerate_way_tags": degenerate_way_tags,
                "removed_relation_members": removed_relation_members,
                "meta_info": {"format_version": "1.0", "map_version": "1.0"},
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
    export(args.input, args.output, args.report)


if __name__ == "__main__":
    main()
