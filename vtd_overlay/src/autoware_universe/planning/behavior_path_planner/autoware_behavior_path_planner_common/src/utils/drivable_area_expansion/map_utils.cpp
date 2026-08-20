// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/map_utils.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
SegmentRtree extract_uncrossable_segments(
  const lanelet::LaneletMap & lanelet_map, const Point & ego_point,
  const DrivableAreaExpansionParameters & params)
{
  SegmentRtree uncrossable_segments_in_range;
  LineString2d line;
  const auto ego_p = Point2d{ego_point.x, ego_point.y};
  const auto linestrings = [&]() {
    if (params.max_path_arc_length <= 0.0) {
      return lanelet::ConstLineStrings3d{
        lanelet_map.lineStringLayer.begin(), lanelet_map.lineStringLayer.end()};
    }
    const auto range = params.max_path_arc_length;
    const lanelet::BoundingBox2d search_area{
      lanelet::BasicPoint2d{ego_point.x - range, ego_point.y - range},
      lanelet::BasicPoint2d{ego_point.x + range, ego_point.y + range}};
    return lanelet_map.lineStringLayer.search(search_area);
  }();

  for (const auto & ls : linestrings) {
    if (has_types(ls, params.avoid_linestring_types)) {
      line.clear();
      for (const auto & p : ls) line.push_back(Point2d{p.x(), p.y()});
      for (auto segment_idx = 0LU; segment_idx + 1 < line.size(); ++segment_idx) {
        Segment2d segment = {line[segment_idx], line[segment_idx + 1]};
        if (
          params.max_path_arc_length <= 0.0 ||
          boost::geometry::distance(segment, ego_p) < params.max_path_arc_length) {
          uncrossable_segments_in_range.insert(segment);
        }
      }
    }
  }
  return uncrossable_segments_in_range;
}

bool has_types(
  const lanelet::ConstLineString3d & ls,
  const std::vector<DrivableAreaExpansionParameters::LinestringType> & types)
{
  constexpr auto no_type = "";
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
  const auto subtype = ls.attributeOr(lanelet::AttributeName::Subtype, no_type);
  const auto matches_type = [&](const auto & t) { return t.matches(type, subtype); };
  return (type != no_type && std::find_if(types.begin(), types.end(), matches_type) != types.end());
}
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
