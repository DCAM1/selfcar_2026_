#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT_PATH = Path(__file__).with_name("set_route_from_csv.py")
SPEC = importlib.util.spec_from_file_location("set_route_from_csv", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
ROUTE_MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ROUTE_MODULE
SPEC.loader.exec_module(ROUTE_MODULE)


class RouteCsvTest(unittest.TestCase):
    def parse(self, text: str):
        with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as route_file:
            route_file.write(text)
            path = Path(route_file.name)
        try:
            return ROUTE_MODULE.load_route_csv(path)
        finally:
            path.unlink()

    def test_orders_points_and_assigns_roles(self) -> None:
        points = self.parse("seq,x,y\n3,8,6\n1,0,0\n2,0,6\n")

        self.assertEqual([point.seq for point in points], [1, 2, 3])
        self.assertEqual(ROUTE_MODULE.route_summary(points), (
            "start=seq 1 (0.000, 0.000), checkpoints=1, "
            "goal=seq 3 (8.000, 6.000)"
        ))
        self.assertAlmostEqual(ROUTE_MODULE.point_yaw(points, 0), math.pi / 2.0)
        self.assertAlmostEqual(
            ROUTE_MODULE.point_yaw(points, 1), math.atan2(6.0, 8.0)
        )
        self.assertAlmostEqual(ROUTE_MODULE.point_yaw(points, 2), 0.0)

    def test_optional_z_and_yaw_override(self) -> None:
        points = self.parse("seq,x,y,z,yaw\n1,0,0,1.5,0.25\n2,2,0,,\n")

        self.assertEqual(points[0].z, 1.5)
        self.assertEqual(points[1].z, 0.0)
        self.assertEqual(ROUTE_MODULE.point_yaw(points, 0), 0.25)
        self.assertEqual(ROUTE_MODULE.point_yaw(points, 1), 0.0)

    def test_optional_lanelet_override(self) -> None:
        points = self.parse(
            "seq,x,y,lanelet_id\n1,0,0,42181\n2,2,0,38867\n"
        )

        self.assertEqual(points[0].lanelet_id, 42181)
        self.assertEqual(points[1].lanelet_id, 38867)

    def test_rejects_missing_sequence_number(self) -> None:
        with self.assertRaisesRegex(
            ROUTE_MODULE.RouteCsvError, "must be consecutive from 1"
        ):
            self.parse("seq,x,y\n1,0,0\n3,1,0\n")

    def test_rejects_duplicate_position(self) -> None:
        with self.assertRaisesRegex(ROUTE_MODULE.RouteCsvError, "same x,y position"):
            self.parse("seq,x,y\n1,0,0\n2,0,0\n")

    def test_rejects_nonfinite_coordinate(self) -> None:
        with self.assertRaisesRegex(ROUTE_MODULE.RouteCsvError, "must be finite"):
            self.parse("seq,x,y\n1,0,0\n2,nan,1\n")

    def test_projects_to_directed_centerline(self) -> None:
        projection = ROUTE_MODULE._project_to_polyline(
            ((0.0, 0.0, 1.0), (10.0, 0.0, 3.0)), 4.0, 2.0
        )

        self.assertAlmostEqual(projection[0], 2.0)
        self.assertAlmostEqual(projection[1], 4.0)
        self.assertAlmostEqual(projection[2], 0.0)
        self.assertAlmostEqual(projection[3], 1.8)
        self.assertAlmostEqual(projection[4], 0.0)
        self.assertAlmostEqual(projection[5], 4.0)
        self.assertAlmostEqual(projection[6], 10.0)


if __name__ == "__main__":
    unittest.main()
