/*
 * scene_node.cpp — ROS2 Humble
 * Scene node: supports builtin highway and EPSILON highway_lite scenarios.
 * Publishes arena info, vehicle states, lanes, and visualization markers.
 */
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lane_change_sim/msg/arena_info_dynamic.hpp"
#include "lane_change_sim/msg/arena_info_static.hpp"
#include "lane_change_sim/msg/lane_array.hpp"
#include "lane_change_sim/msg/vehicle_array.hpp"
#include "lane_change_sim/msg/vehicle_set.hpp"
#include "lane_change_sim/srv/reset_scenario.hpp"
#include "lane_change_sim/ros/msg_adapter.hpp"
#include "lane_change_sim/scenario/scenario_loader.hpp"
#include "lane_change_sim/types.hpp"

using namespace lcs;

// ─────────────── Scene state ───────────────

struct SceneState {
  std::vector<Vehicle> vehicles;
  std::vector<Lane> lanes;
  uint32_t target_lane_id{2};
  double target_x{200.0};
  std::string scenario_source{"builtin_highway"};
  ScenarioConfig scenario_config;
};

// ─────────────── Visualization helpers ───────────────

visualization_msgs::msg::Marker makeDashedLine(const std::vector<geometry_msgs::msg::Point>& pts,
                                               const std::string& ns, int id,
                                               float r, float g, float b, float a,
                                               float width,
                                               double dash_len, double gap_len,
                                               const rclcpp::Time& stamp) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = stamp;
  m.ns = ns; m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = width;
  m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
  m.pose.orientation.w = 1.0;
  if (pts.size() < 2) return m;
  double acc = 0.0;
  bool in_dash = true;
  for (size_t i = 1; i < pts.size(); ++i) {
    const double dx = pts[i].x - pts[i-1].x;
    const double dy = pts[i].y - pts[i-1].y;
    double seg_len = std::hypot(dx, dy);
    if (seg_len < 1e-9) continue;
    double remaining = seg_len;
    double frac_start = 0.0;
    while (remaining > 1e-9) {
      const double phase_len = in_dash ? dash_len : gap_len;
      const double step = std::min(remaining, phase_len - acc);
      const double frac_end = frac_start + step / seg_len;
      if (in_dash) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = pts[i-1].x + frac_start * dx;
        p1.y = pts[i-1].y + frac_start * dy;
        p1.z = 0.0;
        p2.x = pts[i-1].x + frac_end * dx;
        p2.y = pts[i-1].y + frac_end * dy;
        p2.z = 0.0;
        m.points.push_back(p1);
        m.points.push_back(p2);
      }
      acc += step;
      frac_start = frac_end;
      remaining -= step;
      if (acc >= phase_len - 1e-9) {
        acc = 0.0;
        in_dash = !in_dash;
      }
    }
  }
  return m;
}

visualization_msgs::msg::Marker makeLaneMarker(const Lane& lane, const std::string& ns, int id,
                                               const rclcpp::Time& stamp) {
  std::vector<geometry_msgs::msg::Point> pts;
  if (!lane.points.empty()) {
    for (const auto& p : lane.points) pts.push_back(ros_adapter::toRosPoint(p));
  } else {
    for (double x = -4.0; x <= 220.0; x += 2.0) {
      geometry_msgs::msg::Point p; p.x = x; p.y = lane.center_y; p.z = 0.0;
      pts.push_back(p);
    }
  }
  return makeDashedLine(pts, ns, id, 1.0f, 1.0f, 1.0f, 0.6f, 0.1f, 2.0, 1.5, stamp);
}

visualization_msgs::msg::Marker makeLaneBound(const Lane& lane, bool left, bool is_dashed,
                                              const std::string& ns, int id,
                                              const rclcpp::Time& stamp) {
  std::vector<geometry_msgs::msg::Point> pts;
  if (!lane.points.empty()) {
    const double dy = (left ? lane.left_y : lane.right_y) - lane.center_y;
    for (const auto& p : lane.points) {
      geometry_msgs::msg::Point gp = ros_adapter::toRosPoint(p);
      gp.y += dy;
      pts.push_back(gp);
    }
  } else {
    const double y = left ? lane.left_y : lane.right_y;
    for (double x = -4.0; x <= 220.0; x += 2.0) {
      geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = 0.0;
      pts.push_back(p);
    }
  }
  if (is_dashed) {
    return makeDashedLine(pts, ns, id, 0.9f, 0.9f, 0.0f, 0.9f, 0.12f, 1.2, 0.8, stamp);
  } else {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map"; m.header.stamp = stamp;
    m.ns = ns; m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.18;
    m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;
    m.points = pts;
    return m;
  }
}

// ─────────────── SceneNode class ───────────────

class SceneNode : public rclcpp::Node {
 public:
  SceneNode()
    : Node("scene_node"),
      static_dirty_(true) {
    // Parameters
    this->declare_parameter<std::string>("scenario_mode", "builtin_highway");
    this->declare_parameter<std::string>("epsilon_root", "");
    this->declare_parameter<double>("epsilon_lane_width", 4.0);
    this->declare_parameter<double>("epsilon_ref_speed", 7.0);
    this->declare_parameter<int>("epsilon_max_vehicles", 7);
    this->declare_parameter<int>("target_lane_id", 2);
    this->declare_parameter<double>("target_x", 200.0);
    this->declare_parameter<double>("publish_rate", 10.0);

    // Load scenario config
    scene_.scenario_config.mode = this->get_parameter("scenario_mode").as_string();
    scene_.scenario_config.epsilon_root = this->get_parameter("epsilon_root").as_string();
    scene_.scenario_config.lane_width = this->get_parameter("epsilon_lane_width").as_double();
    scene_.scenario_config.ref_speed = this->get_parameter("epsilon_ref_speed").as_double();
    scene_.scenario_config.max_vehicles = static_cast<std::size_t>(
        std::max<int64_t>(1, this->get_parameter("epsilon_max_vehicles").as_int()));
    scene_.target_lane_id = static_cast<uint32_t>(
        std::max<int64_t>(1, this->get_parameter("target_lane_id").as_int()));
    scene_.target_x = this->get_parameter("target_x").as_double();
    scene_.scenario_config.target_lane_id = scene_.target_lane_id;
    scene_.scenario_config.target_x = scene_.target_x;

    if (!loadScene()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize scenario from mode=%s.",
                   scene_.scenario_config.mode.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Initialized scenario=%s with %zu vehicles and %zu lanes.",
                scene_.scenario_source.c_str(), scene_.vehicles.size(), scene_.lanes.size());

    // Publishers
    veh_pub_ = this->create_publisher<lane_change_sim::msg::VehicleArray>("/scene/vehicle_states", 10);
    vehicle_set_pub_ = this->create_publisher<lane_change_sim::msg::VehicleSet>("/scene/vehicle_set",
        rclcpp::QoS(10).transient_local());
    lane_pub_ = this->create_publisher<lane_change_sim::msg::LaneArray>("/scene/lanes",
        rclcpp::QoS(10).transient_local());
    arena_static_pub_ = this->create_publisher<lane_change_sim::msg::ArenaInfoStatic>("/scene/arena_info_static",
        rclcpp::QoS(10).transient_local());
    arena_dynamic_pub_ = this->create_publisher<lane_change_sim::msg::ArenaInfoDynamic>("/scene/arena_info_dynamic", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/scene/visualization", 10);

    // Service
    reset_srv_ = this->create_service<lane_change_sim::srv::ResetScenario>("/scene/reset",
        [this](const std::shared_ptr<lane_change_sim::srv::ResetScenario::Request> /*req*/,
               std::shared_ptr<lane_change_sim::srv::ResetScenario::Response> res) {
          if (!loadScene()) {
            res->success = false;
            res->message = "Scene reset failed";
            RCLCPP_ERROR(this->get_logger(), "Scene reset failed.");
            return;
          }
          static_dirty_ = true;
          res->success = true;
          res->message = "Scene reset OK";
          RCLCPP_INFO(this->get_logger(), "Scene reset from source=%s.", scene_.scenario_source.c_str());
        });

    // Timer
    const double rate_hz = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz),
        std::bind(&SceneNode::timerCallback, this));
  }

 private:
  bool loadScene() {
    std::vector<Lane> lanes;
    std::vector<Vehicle> vehicles;
    std::string source;
    if (!loader_.load(scene_.scenario_config, &lanes, &vehicles, &source)) {
      return false;
    }
    scene_.lanes = std::move(lanes);
    scene_.vehicles = std::move(vehicles);
    scene_.scenario_source = std::move(source);
    return true;
  }

  void timerCallback() {
    auto now = this->now();

    if (static_dirty_) {
      lane_pub_->publish(ros_adapter::toRosLaneArray(scene_.lanes, now, "map"));
      vehicle_set_pub_->publish(ros_adapter::toRosVehicleSet(scene_.vehicles, now, "map"));
      arena_static_pub_->publish(ros_adapter::toRosArenaInfoStatic(
          scene_.lanes, scene_.scenario_source, scene_.target_lane_id, scene_.target_x, now, "map"));
      static_dirty_ = false;
    }

    veh_pub_->publish(ros_adapter::toRosVehicleArray(scene_.vehicles, now, "map"));
    arena_dynamic_pub_->publish(ros_adapter::toRosArenaInfoDynamic(scene_.vehicles, now, "map"));

    // Visualization markers
    visualization_msgs::msg::MarkerArray ma;
    int mid = 0;
    for (const auto& l : scene_.lanes) {
      ma.markers.push_back(makeLaneMarker(l, "lane_center", mid++, now));
      ma.markers.push_back(makeLaneBound(l, true,  l.left_change_available,  "lane_bounds", mid++, now));
      ma.markers.push_back(makeLaneBound(l, false, l.right_change_available, "lane_bounds", mid++, now));
    }
    viz_pub_->publish(ma);
  }

  SceneState scene_;
  ScenarioLoader loader_;
  bool static_dirty_;

  rclcpp::Publisher<lane_change_sim::msg::VehicleArray>::SharedPtr veh_pub_;
  rclcpp::Publisher<lane_change_sim::msg::VehicleSet>::SharedPtr vehicle_set_pub_;
  rclcpp::Publisher<lane_change_sim::msg::LaneArray>::SharedPtr lane_pub_;
  rclcpp::Publisher<lane_change_sim::msg::ArenaInfoStatic>::SharedPtr arena_static_pub_;
  rclcpp::Publisher<lane_change_sim::msg::ArenaInfoDynamic>::SharedPtr arena_dynamic_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Service<lane_change_sim::srv::ResetScenario>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneNode>());
  rclcpp::shutdown();
  return 0;
}
