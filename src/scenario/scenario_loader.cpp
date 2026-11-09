#include "lane_change_sim/scenario/scenario_loader.hpp"
#include "lane_change_sim/utils/lane_utils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace lcs {
namespace {

namespace pt = boost::property_tree;

struct RawLaneSegment {
  int source_id{0};
  int left_id{0};
  int right_id{0};
  bool left_change_available{false};
  bool right_change_available{false};
  std::string behavior{"s"};
  double mean_y{0.0};
  std::vector<Point2d> points;
};

struct RawVehicleRecord {
  Vehicle vehicle;
};

struct PoseTransform {
  double cos_yaw{1.0};
  double sin_yaw{0.0};
  Point2d origin;
  double x_shift{0.0};
};

Point2d parsePoint2d(const pt::ptree& node) {
  auto it = node.begin();
  Point2d point;
  if (it != node.end()) {
    point.x = it->second.get_value<double>();
    ++it;
  }
  if (it != node.end()) {
    point.y = it->second.get_value<double>();
  }
  return point;
}

Point2d transformPoint(const Point2d& world, const PoseTransform& tf) {
  const double dx = world.x - tf.origin.x;
  const double dy = world.y - tf.origin.y;
  Point2d local;
  local.x = tf.cos_yaw * dx + tf.sin_yaw * dy + tf.x_shift;
  local.y = -tf.sin_yaw * dx + tf.cos_yaw * dy;
  return local;
}

Vehicle makeVehicle(uint32_t id, double px, double py, double speed, uint32_t lane) {
  Vehicle vehicle;
  vehicle.id = id;
  vehicle.lane_id = lane;
  vehicle.state.px = px;
  vehicle.state.py = py;
  vehicle.state.speed = speed;
  vehicle.subclass = "car";
  vehicle.type = "VehicleWithBicycleKinematics";
  return vehicle;
}

double meanLaneY(const std::vector<Point2d>& points) {
  if (points.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto& point : points) {
    sum += point.y;
  }
  return sum / static_cast<double>(points.size());
}

double laneLength(const std::vector<Point2d>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double total = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    total += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  }
  return total;
}

bool readJson(const std::string& path, pt::ptree* tree) {
  std::ifstream stream(path);
  if (!stream.is_open()) {
    return false;
  }
  pt::read_json(stream, *tree);
  return true;
}

}  // namespace

bool ScenarioLoader::load(const ScenarioConfig& config,
                          std::vector<Lane>* lanes,
                          std::vector<Vehicle>* vehicles,
                          std::string* scenario_source) const {
  if (config.mode == "epsilon_highway_lite") {
    if (loadEpsilonHighway(config, lanes, vehicles, scenario_source)) {
      return true;
    }
  }
  return loadBuiltinHighway(config, lanes, vehicles, scenario_source);
}

bool ScenarioLoader::loadBuiltinHighway(const ScenarioConfig& config,
                                        std::vector<Lane>* lanes,
                                        std::vector<Vehicle>* vehicles,
                                        std::string* scenario_source) const {
  lanes->clear();
  vehicles->clear();

  std::vector<double> centers = {2.0, 6.0, 10.0, 14.0};
  for (size_t i = 0; i < centers.size(); ++i) {
    Lane lane;
    lane.id = static_cast<uint32_t>(i + 1);
    lane.dir = 1;
    lane.left_lane_id = (i + 1 < centers.size()) ? static_cast<int>(i + 2) : 0;
    lane.right_lane_id = (i > 0) ? static_cast<int>(i) : 0;
    lane.left_change_available = lane.left_lane_id != 0;
    lane.right_change_available = lane.right_lane_id != 0;
    lane.behavior = "s";
    lane.center_y = centers[i];
    lane.left_y = centers[i] + config.lane_width * 0.5;
    lane.right_y = centers[i] - config.lane_width * 0.5;
    lane.ref_speed = config.ref_speed;
    lane.length = config.target_x + 20.0;
    for (double x = -4.0; x <= config.target_x + 20.0; x += 4.0) {
      lane.points.push_back(Point2d{x, lane.center_y});
    }
    lanes->push_back(lane);
  }

  vehicles->push_back(makeVehicle(0,   5.0, 10.0,  5.0, 3));
  vehicles->push_back(makeVehicle(1,  25.0, 10.0,  3.0, 3));
  vehicles->push_back(makeVehicle(2,  30.0, 14.0,  5.0, 4));
  vehicles->push_back(makeVehicle(3, -20.0, 14.0,  5.0, 4));
  vehicles->push_back(makeVehicle(4,  40.0,  6.0,  5.0, 2));
  vehicles->push_back(makeVehicle(5, -15.0,  6.0,  7.0, 2));
  vehicles->push_back(makeVehicle(6,  20.0,  2.0,  5.0, 1));

  if (scenario_source) {
    *scenario_source = "builtin_highway";
  }
  return true;
}

bool ScenarioLoader::loadEpsilonHighway(const ScenarioConfig& config,
                                        std::vector<Lane>* lanes,
                                        std::vector<Vehicle>* vehicles,
                                        std::string* scenario_source) const {
  if (config.epsilon_root.empty()) {
    return false;
  }

  pt::ptree lane_tree;
  pt::ptree vehicle_tree;
  const std::string lane_path = config.epsilon_root + "/lane_net_norm.json";
  const std::string vehicle_path = config.epsilon_root + "/vehicle_set.json";
  if (!readJson(lane_path, &lane_tree) || !readJson(vehicle_path, &vehicle_tree)) {
    return false;
  }

  std::vector<RawLaneSegment> raw_segments;
  for (const auto& feature_entry : lane_tree.get_child("features")) {
    const auto& feature = feature_entry.second;
    RawLaneSegment segment;
    const auto& properties = feature.get_child("properties");
    segment.source_id = properties.get<int>("id", 0);
    segment.left_id = properties.get<int>("left_id", 0);
    segment.right_id = properties.get<int>("right_id", 0);
    segment.left_change_available = properties.get<int>("lchg_vld", 0) != 0;
    segment.right_change_available = properties.get<int>("rchg_vld", 0) != 0;
    segment.behavior = properties.get<std::string>("behavior", "s");

    const auto& coordinates = feature.get_child("geometry.coordinates");
    if (coordinates.empty()) {
      continue;
    }
    const auto& multiline = coordinates.begin()->second;
    for (const auto& point_entry : multiline) {
      segment.points.push_back(parsePoint2d(point_entry.second));
    }
    if (segment.points.size() < 2) {
      continue;
    }
    raw_segments.push_back(segment);
  }
  if (raw_segments.empty()) {
    return false;
  }

  auto ref_it = std::max_element(raw_segments.begin(), raw_segments.end(),
      [](const RawLaneSegment& a, const RawLaneSegment& b) {
        return laneLength(a.points) < laneLength(b.points);
      });
  const Point2d ref_start = ref_it->points.front();
  const Point2d ref_end = ref_it->points.back();
  const double heading = std::atan2(ref_end.y - ref_start.y, ref_end.x - ref_start.x);
  const PoseTransform transform{std::cos(heading), std::sin(heading), ref_start, 0.0};

  for (auto& segment : raw_segments) {
    for (auto& point : segment.points) {
      point = transformPoint(point, transform);
    }
    segment.mean_y = meanLaneY(segment.points);
  }

  std::sort(raw_segments.begin(), raw_segments.end(), [](const RawLaneSegment& a, const RawLaneSegment& b) {
    return a.mean_y < b.mean_y;
  });

  const double cluster_threshold = std::max(1.5, config.lane_width * 0.6);
  std::vector<std::vector<RawLaneSegment>> clusters;
  for (const auto& segment : raw_segments) {
    if (clusters.empty() || std::abs(segment.mean_y - clusters.back().back().mean_y) > cluster_threshold) {
      clusters.push_back({segment});
    } else {
      clusters.back().push_back(segment);
    }
  }
  if (clusters.empty()) {
    return false;
  }

  lanes->clear();
  lanes->reserve(clusters.size());
  std::vector<double> cluster_mean_y;
  cluster_mean_y.reserve(clusters.size());

  for (size_t idx = 0; idx < clusters.size(); ++idx) {
    auto& cluster = clusters[idx];
    std::sort(cluster.begin(), cluster.end(), [](const RawLaneSegment& a, const RawLaneSegment& b) {
      return a.points.front().x < b.points.front().x;
    });

    Lane lane;
    lane.id = static_cast<uint32_t>(idx + 1);
    lane.dir = 1;
    lane.behavior = cluster.front().behavior;
    lane.left_lane_id = (idx + 1 < clusters.size()) ? static_cast<int>(idx + 2) : 0;
    lane.right_lane_id = (idx > 0) ? static_cast<int>(idx) : 0;
    lane.left_change_available = lane.left_lane_id != 0;
    lane.right_change_available = lane.right_lane_id != 0;
    lane.center_y = 2.0 + static_cast<double>(idx) * config.lane_width;
    lane.left_y = lane.center_y + config.lane_width * 0.5;
    lane.right_y = lane.center_y - config.lane_width * 0.5;
    lane.ref_speed = config.ref_speed;

    const double source_mean_y = std::accumulate(cluster.begin(), cluster.end(), 0.0,
        [](double sum, const RawLaneSegment& item) { return sum + item.mean_y; }) / static_cast<double>(cluster.size());
    cluster_mean_y.push_back(source_mean_y);

    for (const auto& segment : cluster) {
      lane.child_ids.push_back(static_cast<uint32_t>(std::max(segment.left_id, 0)));
      lane.father_ids.push_back(static_cast<uint32_t>(std::max(segment.right_id, 0)));
      for (const auto& point : segment.points) {
        Point2d normalized{point.x, lane.center_y + (point.y - source_mean_y)};
        if (lane.points.empty() || std::hypot(normalized.x - lane.points.back().x, normalized.y - lane.points.back().y) > 0.2) {
          lane.points.push_back(normalized);
        }
      }
    }
    lane.length = laneLength(lane.points);
    lanes->push_back(lane);
  }

  std::vector<RawVehicleRecord> raw_vehicles;
  for (const auto& vehicle_entry : vehicle_tree.get_child("vehicles.info")) {
    const auto& node = vehicle_entry.second;
    RawVehicleRecord record;
    record.vehicle.id = node.get<uint32_t>("id", 0);
    record.vehicle.subclass = node.get<std::string>("subclass", "car");
    record.vehicle.type = node.get<std::string>("type", "VehicleWithBicycleKinematics");

    const auto& state = node.get_child("init_state");
    Point2d pos{state.get<double>("x", 0.0), state.get<double>("y", 0.0)};
    const auto local = transformPoint(pos, transform);
    record.vehicle.state.px = local.x;
    record.vehicle.state.py = local.y;
    record.vehicle.state.heading = state.get<double>("angle", 0.0) - heading;
    record.vehicle.state.speed = std::max(4.0, state.get<double>("velocity", 0.0));
    record.vehicle.acc = state.get<double>("acceleration", 0.0);
    record.vehicle.steer = state.get<double>("steer", 0.0);

    const auto& param = node.get_child("params");
    record.vehicle.width = param.get<double>("width", record.vehicle.width);
    record.vehicle.length = param.get<double>("length", record.vehicle.length);
    record.vehicle.wheel_base = param.get<double>("wheel_base", record.vehicle.wheel_base);
    record.vehicle.front_suspension = param.get<double>("front_suspension", record.vehicle.front_suspension);
    record.vehicle.rear_suspension = param.get<double>("rear_suspension", record.vehicle.rear_suspension);
    record.vehicle.max_steering_angle = param.get<double>("max_steering_angle", 45.0) * M_PI / 180.0;
    record.vehicle.max_longitudinal_acc = param.get<double>("max_longitudinal_acc", record.vehicle.max_longitudinal_acc);
    record.vehicle.max_lateral_acc = param.get<double>("max_lateral_acc", record.vehicle.max_lateral_acc);
    raw_vehicles.push_back(record);
  }
  if (raw_vehicles.empty()) {
    return false;
  }

  auto ego_it = std::find_if(raw_vehicles.begin(), raw_vehicles.end(), [](const RawVehicleRecord& record) {
    return record.vehicle.id == 0;
  });
  if (ego_it == raw_vehicles.end()) {
    ego_it = raw_vehicles.begin();
  }
  const double ego_x = ego_it->vehicle.state.px;
  for (auto& lane : *lanes) {
    for (auto& point : lane.points) {
      point.x = point.x - ego_x + 5.0;
    }
    lane.length = laneLength(lane.points);
  }

  for (auto& record : raw_vehicles) {
    record.vehicle.state.px = record.vehicle.state.px - ego_x + 5.0;
    auto best_idx = size_t{0};
    double best = std::numeric_limits<double>::max();
    for (size_t idx = 0; idx < cluster_mean_y.size(); ++idx) {
      const double dist = std::abs(record.vehicle.state.py - cluster_mean_y[idx]);
      if (dist < best) {
        best = dist;
        best_idx = idx;
      }
    }
    record.vehicle.state.py = lanes->at(best_idx).center_y + (record.vehicle.state.py - cluster_mean_y[best_idx]);
    record.vehicle.lane_id = utils::nearestLaneId(record.vehicle.state.py, *lanes);
  }

  std::sort(raw_vehicles.begin(), raw_vehicles.end(), [&](const RawVehicleRecord& a, const RawVehicleRecord& b) {
    const double da = std::abs(a.vehicle.state.px - 5.0);
    const double db = std::abs(b.vehicle.state.px - 5.0);
    if (a.vehicle.id == 0) {
      return true;
    }
    if (b.vehicle.id == 0) {
      return false;
    }
    return da < db;
  });

  vehicles->clear();
  const std::size_t keep = std::min(config.max_vehicles, raw_vehicles.size());
  vehicles->reserve(keep);
  bool ego_included = false;
  for (size_t i = 0; i < keep; ++i) {
    vehicles->push_back(raw_vehicles[i].vehicle);
    ego_included = ego_included || raw_vehicles[i].vehicle.id == 0;
  }
  if (!ego_included) {
    vehicles->back() = ego_it->vehicle;
    vehicles->back().state.px = ego_it->vehicle.state.px - ego_x + 5.0;
    vehicles->back().lane_id = utils::nearestLaneId(vehicles->back().state.py, *lanes);
  }

  std::sort(vehicles->begin(), vehicles->end(), [](const Vehicle& a, const Vehicle& b) {
    return a.id < b.id;
  });

  if (scenario_source) {
    *scenario_source = "epsilon_highway_lite";
  }
  return !lanes->empty() && !vehicles->empty();
}

}  // namespace lcs
