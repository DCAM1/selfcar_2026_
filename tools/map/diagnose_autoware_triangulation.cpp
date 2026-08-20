#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>

#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: diagnose_autoware_triangulation MAP.osm\n";
    return 2;
  }

  lanelet::ErrorMessages errors;
  auto projector = std::make_unique<lanelet::projection::LocalCartesianProjector>(
    lanelet::Origin({0.0, 0.0}));
  auto map = lanelet::load(argv[1], *projector, &errors);
  for (const auto & error : errors) {
    std::cerr << "load: " << error << '\n';
  }

  std::size_t count = 0;
  for (const auto & lanelet : map->laneletLayer) {
    std::cerr << "BEGIN_LANELET " << lanelet.id() << '\n';
    std::vector<geometry_msgs::msg::Polygon> triangles;
    lanelet::visualization::lanelet2Triangle(lanelet, &triangles);
    std::cerr << "END_LANELET " << lanelet.id() << " triangles=" << triangles.size() << '\n';
    ++count;
  }
  std::cerr << "TOTAL_LANELETS " << count << '\n';

  const lanelet::LaneletMapConstPtr const_map(map.release());
  const auto msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(const_map);
  const auto round_trip_const =
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(msg);
  const auto round_trip_map =
    autoware::experimental::lanelet2_utils::remove_const(round_trip_const);
  const lanelet::LaneletMapConstPtr viz_map = round_trip_map;
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_map);
  std::cerr << "BEGIN_ROUND_TRIP_LANELETS " << all_lanelets.size() << '\n';
  for (const auto & lanelet : all_lanelets) {
    std::cerr << "BEGIN_ROUND_LANELET " << lanelet.id() << '\n';
    std::vector<geometry_msgs::msg::Polygon> triangles;
    lanelet::visualization::lanelet2Triangle(lanelet, &triangles);
    std::cerr << "END_ROUND_LANELET " << lanelet.id() << '\n';
  }
  std::cerr << "END_ROUND_TRIP_LANELETS\n";
  const auto road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  const auto walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  const auto traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std_msgs::msg::ColorRGBA color;
  color.r = color.g = color.b = color.a = 1.0F;

  std::cerr << "BEGIN_GROUP road count=" << road_lanelets.size() << '\n';
  (void)lanelet::visualization::laneletsAsTriangleMarkerArray("road", road_lanelets, color);
  std::cerr << "END_GROUP road\n";
  std::cerr << "BEGIN_GROUP walkway count=" << walkway_lanelets.size() << '\n';
  (void)lanelet::visualization::laneletsAsTriangleMarkerArray("walkway", walkway_lanelets, color);
  std::cerr << "END_GROUP walkway\n";
  std::cerr << "BEGIN_GROUP traffic_light count=" << traffic_lights.size() << '\n';
  (void)lanelet::visualization::autowareTrafficLightsAsMarkerArray(traffic_lights, color);
  std::cerr << "END_GROUP traffic_light\n";
  return 0;
}
