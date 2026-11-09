#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lane_change_sim/msg/arena_info_dynamic.hpp"
#include "lane_change_sim/msg/arena_info_static.hpp"
#include "lane_change_sim/msg/behavior_decision.hpp"
#include "lane_change_sim/msg/lane.hpp"
#include "lane_change_sim/msg/lane_array.hpp"
#include "lane_change_sim/msg/lane_net.hpp"
#include "lane_change_sim/msg/state.hpp"
#include "lane_change_sim/msg/trajectory.hpp"
#include "lane_change_sim/msg/trajectory_point.hpp"
#include "lane_change_sim/msg/vehicle.hpp"
#include "lane_change_sim/msg/vehicle_array.hpp"
#include "lane_change_sim/msg/vehicle_param.hpp"
#include "lane_change_sim/msg/vehicle_set.hpp"
#include "lane_change_sim/msg/vehicle_state.hpp"
#include "lane_change_sim/types.hpp"

namespace lcs::ros_adapter {

inline geometry_msgs::msg::Point toRosPoint(const Point2d& p) {
  geometry_msgs::msg::Point msg;
  msg.x = p.x;
  msg.y = p.y;
  msg.z = 0.0;
  return msg;
}

inline Point2d fromRosPoint(const geometry_msgs::msg::Point& p) {
  return Point2d{p.x, p.y};
}

inline lane_change_sim::msg::VehicleParam toRosVehicleParam(const Vehicle& v) {
  lane_change_sim::msg::VehicleParam msg;
  msg.width = v.width;
  msg.length = v.length;
  msg.wheel_base = v.wheel_base;
  msg.front_suspension = v.front_suspension;
  msg.rear_suspension = v.rear_suspension;
  msg.max_steering_angle = v.max_steering_angle;
  msg.max_longitudinal_acc = v.max_longitudinal_acc;
  msg.max_lateral_acc = v.max_lateral_acc;
  return msg;
}

inline lane_change_sim::msg::State toRosState(const VehicleState& state,
                                              const rclcpp::Time& stamp) {
  lane_change_sim::msg::State msg;
  msg.header.stamp = stamp;
  msg.vec_position.x = state.px;
  msg.vec_position.y = state.py;
  msg.angle = state.heading;
  msg.curvature = 0.0;
  msg.velocity = state.speed;
  msg.acceleration = 0.0;
  msg.steer = 0.0;
  return msg;
}

inline lane_change_sim::msg::Vehicle toRosVehicle(const Vehicle& v,
                                                   const rclcpp::Time& stamp) {
  lane_change_sim::msg::Vehicle msg;
  msg.header.stamp = stamp;
  msg.id = static_cast<int32_t>(v.id);
  msg.subclass = v.subclass;
  msg.type = v.type;
  msg.param = toRosVehicleParam(v);
  msg.state = toRosState(v.state, stamp);
  msg.state.acceleration = v.acc;
  msg.state.steer = v.steer;
  return msg;
}

inline lane_change_sim::msg::VehicleState toRosVehicleState(const Vehicle& v,
                                                            const rclcpp::Time& stamp) {
  lane_change_sim::msg::VehicleState msg;
  msg.header.stamp = stamp;
  msg.id = v.id;
  msg.lane_id = v.lane_id;
  msg.px = v.state.px;
  msg.py = v.state.py;
  msg.speed = v.state.speed;
  msg.heading = v.state.heading;
  msg.yaw_rate = v.state.yaw_rate;
  msg.acc = v.acc;
  msg.steer = v.steer;
  msg.lat_behavior = static_cast<uint8_t>(v.lat_behavior);
  msg.lon_behavior = static_cast<uint8_t>(v.lon_behavior);
  msg.lane_change_stage = static_cast<uint8_t>(v.lane_change_stage);
  msg.intention = static_cast<uint8_t>(v.intention);
  msg.desired_speed = v.desired_speed;
  return msg;
}

inline Vehicle fromRosVehicleState(const lane_change_sim::msg::VehicleState& msg) {
  Vehicle v;
  v.id = msg.id;
  v.lane_id = msg.lane_id;
  v.state.px = msg.px;
  v.state.py = msg.py;
  v.state.speed = msg.speed;
  v.state.heading = msg.heading;
  v.state.yaw_rate = msg.yaw_rate;
  v.acc = msg.acc;
  v.steer = msg.steer;
  v.lat_behavior = static_cast<LatBehavior>(msg.lat_behavior);
  v.lon_behavior = static_cast<LonBehavior>(msg.lon_behavior);
  v.lane_change_stage = static_cast<LaneChangeStage>(msg.lane_change_stage);
  v.intention = static_cast<Intention>(msg.intention);
  v.desired_speed = msg.desired_speed;
  return v;
}

inline lane_change_sim::msg::Lane toRosLane(const Lane& lane) {
  lane_change_sim::msg::Lane msg;
  msg.id = lane.id;
  msg.dir = lane.dir;
  msg.child_id.assign(lane.child_ids.begin(), lane.child_ids.end());
  msg.father_id.assign(lane.father_ids.begin(), lane.father_ids.end());
  msg.l_lane_id = lane.left_lane_id;
  msg.l_change_avbl = lane.left_change_available;
  msg.r_lane_id = lane.right_lane_id;
  msg.r_change_avbl = lane.right_change_available;
  msg.behavior = lane.behavior;
  msg.length = lane.length;
  msg.center_y = lane.center_y;
  msg.left_bound = lane.left_y;
  msg.right_bound = lane.right_y;
  msg.ref_speed = lane.ref_speed;
  if (!lane.points.empty()) {
    msg.start_point = toRosPoint(lane.points.front());
    msg.final_point = toRosPoint(lane.points.back());
  }
  for (const auto& point : lane.points) {
    msg.points.push_back(toRosPoint(point));
  }
  return msg;
}

inline Lane fromRosLane(const lane_change_sim::msg::Lane& msg) {
  Lane lane;
  lane.id = msg.id;
  lane.dir = msg.dir;
  lane.child_ids.assign(msg.child_id.begin(), msg.child_id.end());
  lane.father_ids.assign(msg.father_id.begin(), msg.father_id.end());
  lane.left_lane_id = msg.l_lane_id;
  lane.left_change_available = msg.l_change_avbl;
  lane.right_lane_id = msg.r_lane_id;
  lane.right_change_available = msg.r_change_avbl;
  lane.behavior = msg.behavior;
  lane.length = msg.length;
  lane.center_y = msg.center_y;
  lane.left_y = msg.left_bound;
  lane.right_y = msg.right_bound;
  lane.ref_speed = msg.ref_speed;
  lane.points.reserve(msg.points.size());
  for (const auto& point : msg.points) {
    lane.points.push_back(fromRosPoint(point));
  }
  return lane;
}

inline lane_change_sim::msg::VehicleArray toRosVehicleArray(
    const std::vector<Vehicle>& vehicles,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
  lane_change_sim::msg::VehicleArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  for (const auto& vehicle : vehicles) {
    msg.vehicles.push_back(toRosVehicleState(vehicle, stamp));
  }
  return msg;
}

inline lane_change_sim::msg::Trajectory toRosTrajectory(
    const Trajectory& trajectory,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
  lane_change_sim::msg::Trajectory msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.points.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points) {
    lane_change_sim::msg::TrajectoryPoint ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.v = point.v;
    ros_point.h = point.h;
    ros_point.a = point.a;
    ros_point.t = point.t;
    msg.points.push_back(ros_point);
  }
  msg.infeasibility = trajectory.infeasibility;
  return msg;
}

inline Trajectory fromRosTrajectory(const lane_change_sim::msg::Trajectory& msg) {
  Trajectory trajectory;
  trajectory.points.reserve(msg.points.size());
  for (const auto& ros_point : msg.points) {
    TrajectoryPoint point;
    point.x = ros_point.x;
    point.y = ros_point.y;
    point.v = ros_point.v;
    point.h = ros_point.h;
    point.a = ros_point.a;
    point.t = ros_point.t;
    trajectory.points.push_back(point);
  }
  trajectory.infeasibility = msg.infeasibility;
  return trajectory;
}

inline lane_change_sim::msg::VehicleSet toRosVehicleSet(
    const std::vector<Vehicle>& vehicles,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
  lane_change_sim::msg::VehicleSet msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  for (const auto& vehicle : vehicles) {
    msg.vehicles.push_back(toRosVehicle(vehicle, stamp));
  }
  return msg;
}

inline lane_change_sim::msg::LaneArray toRosLaneArray(
    const std::vector<Lane>& lanes,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
  lane_change_sim::msg::LaneArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  for (const auto& lane : lanes) {
    msg.lanes.push_back(toRosLane(lane));
  }
  return msg;
}

inline lane_change_sim::msg::LaneNet toRosLaneNet(
    const std::vector<Lane>& lanes,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
  lane_change_sim::msg::LaneNet msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  for (const auto& lane : lanes) {
    msg.lanes.push_back(toRosLane(lane));
  }
  return msg;
}

inline lane_change_sim::msg::ArenaInfoStatic toRosArenaInfoStatic(
    const std::vector<Lane>& lanes,
    const std::string& scenario_source,
    uint32_t target_lane_id,
    double target_x,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
  lane_change_sim::msg::ArenaInfoStatic msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.scenario_source = scenario_source;
  msg.target_lane_id = target_lane_id;
  msg.target_x = target_x;
  msg.lane_net = toRosLaneNet(lanes, stamp, frame_id);
  return msg;
}

inline lane_change_sim::msg::ArenaInfoDynamic toRosArenaInfoDynamic(
    const std::vector<Vehicle>& vehicles,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
  lane_change_sim::msg::ArenaInfoDynamic msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.vehicles = toRosVehicleSet(vehicles, stamp, frame_id);
  return msg;
}

}  // namespace lcs::ros_adapter
