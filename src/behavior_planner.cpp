#include "lane_change_sim/behavior_planner.hpp"
#include "lane_change_sim/control/common.hpp"
#include "lane_change_sim/control/idm_controller.hpp"
#include "lane_change_sim/control/kinematics.hpp"
#include "lane_change_sim/control/lat_controller.hpp"
#include "lane_change_sim/control/rss.hpp"
#include "lane_change_sim/utils/collision_utils.hpp"
#include "lane_change_sim/utils/lane_utils.hpp"
#include "lane_change_sim/utils/math_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include <rclcpp/rclcpp.hpp>

namespace lcs {
namespace {

auto getLogger() { return rclcpp::get_logger("BehaviorPlanner"); }

}  // namespace

BehaviorPlanner::BehaviorPlanner() {
  calibration_.max_search_times = 0;
  for (int i = 1; i <= calibration_.tree_hight; ++i) {
    calibration_.max_search_times += static_cast<int>(std::pow(9, i));
  }
}

void BehaviorPlanner::setCalibration(const Calibration& calibration) {
  calibration_ = calibration;
  calibration_.max_search_times = 0;
  for (int i = 1; i <= calibration_.tree_hight; ++i) {
    calibration_.max_search_times += static_cast<int>(std::pow(9, i));
  }
}

std::vector<LatBehavior> BehaviorPlanner::getPotentialBehaviors(const Vehicle& ego,
                                                                const std::vector<Vehicle>& vehicles,
                                                                const std::vector<Lane>& lanes,
                                                                uint32_t target_lane_id) const {
  (void)target_lane_id;
  const Lane* current_lane = control::FindLaneById(lanes, ego.lane_id);
  const bool can_left = current_lane ? current_lane->left_change_available : false;
  const bool can_right = current_lane ? current_lane->right_change_available : false;

  if (ego.lat_behavior == LatBehavior::LaneKeeping) {
    std::vector<LatBehavior> candidates{LatBehavior::LaneKeeping};

    const LatBehavior avoid = checkAvoidNeeded(ego, vehicles, lanes);
    if (avoid != LatBehavior::LaneKeeping) {
      candidates.push_back(avoid);
    }

    if (can_left && current_lane && current_lane->left_lane_id > 0) {
      const uint32_t target_lane_id = static_cast<uint32_t>(current_lane->left_lane_id);
      if (control::CheckLaneChangeRssSafe(ego, vehicles, target_lane_id, calibration_) &&
          evaluateMobilGain(ego, vehicles, lanes, target_lane_id) > calibration_.mobil_gain_threshold) {
        candidates.push_back(LatBehavior::LaneChangeLeft);
      }
    }
    if (can_right && current_lane && current_lane->right_lane_id > 0) {
      const uint32_t target_lane_id = static_cast<uint32_t>(current_lane->right_lane_id);
      if (control::CheckLaneChangeRssSafe(ego, vehicles, target_lane_id, calibration_) &&
          evaluateMobilGain(ego, vehicles, lanes, target_lane_id) > calibration_.mobil_gain_threshold) {
        candidates.push_back(LatBehavior::LaneChangeRight);
      }
    }
    return candidates;
  }

  if (ego.lane_change_stage == LaneChangeStage::Stage2 ||
      ego.lane_change_stage == LaneChangeStage::Aborting) {
    return {ego.lat_behavior};
  }

  if (ego.lat_behavior == LatBehavior::AvoidLeft ||
      ego.lat_behavior == LatBehavior::AvoidRight) {
    const LatBehavior avoid = checkAvoidNeeded(ego, vehicles, lanes);
    if (avoid != LatBehavior::LaneKeeping) {
      return {LatBehavior::LaneKeeping, avoid};
    }
    return {LatBehavior::LaneKeeping};
  }

  return {LatBehavior::LaneKeeping, ego.lat_behavior};
}

double BehaviorPlanner::evaluateMobilGain(const Vehicle& ego,
                                          const std::vector<Vehicle>& vehicles,
                                          const std::vector<Lane>& lanes,
                                          uint32_t target_lane_id) const {
  const auto* current_lane = control::FindLaneById(lanes, ego.lane_id);
  const auto* target_lane = control::FindLaneById(lanes, target_lane_id);
  if (!current_lane || !target_lane) {
    return -1e6;
  }

  Vehicle ego_current = ego;
  Vehicle ego_target = ego;
  ego_target.lane_id = target_lane_id;

  const auto lead_current = control::FindLeadingVehicle(
      ego_current, vehicles, lanes, 20.0, current_lane->ref_speed, 1.0, true);
  const auto lead_target = control::FindLeadingVehicle(
      ego_target, vehicles, lanes, 20.0, target_lane->ref_speed, 1.0, true);

  const double a_ego_old = control::ComputeIidmAcceleration(
      ego_current, lead_current.state, current_lane->ref_speed, calibration_.min_a, calibration_.max_a);
  const double a_ego_new = control::ComputeIidmAcceleration(
      ego_target, lead_target.state, target_lane->ref_speed, calibration_.min_a, calibration_.max_a);

  const Vehicle* target_rear = nullptr;
  const Vehicle* target_front = nullptr;
  control::FindNearestFrontRearInLane(ego_target, vehicles, target_lane_id, &target_front, &target_rear);

  double rear_delta = 0.0;
  if (target_rear) {
    const auto rear_old_lead = control::FindLeadingVehicle(
        *target_rear, vehicles, lanes, 20.0, target_lane->ref_speed, 1.0, false);
    const double a_rear_old = control::ComputeIidmAcceleration(
        *target_rear, rear_old_lead.state, target_lane->ref_speed, calibration_.min_a, calibration_.max_a);

    VehicleState lead_after_lc = ego_target.state;
    if (rear_old_lead.state.px > ego_target.state.px) {
      lead_after_lc = rear_old_lead.state;
    }
    const double a_rear_new = control::ComputeIidmAcceleration(
        *target_rear, lead_after_lc, target_lane->ref_speed, calibration_.min_a, calibration_.max_a);
    if (a_rear_new < calibration_.mobil_rear_acc_threshold) {
      return -1e6;
    }
    rear_delta = a_rear_new - a_rear_old;
  }

  return (a_ego_new - a_ego_old) + calibration_.mobil_politeness * rear_delta;
}

LatBehavior BehaviorPlanner::checkAvoidNeeded(const Vehicle& ego,
                                              const std::vector<Vehicle>& vehicles,
                                              const std::vector<Lane>& lanes) const {
  const Lane* cur_lane = control::FindLaneById(lanes, ego.lane_id);
  if (!cur_lane) return LatBehavior::LaneKeeping;

  const double lane_width = cur_lane->left_y - cur_lane->right_y;
  const double avoid_threshold = lane_width * 0.6;

  for (const auto& other : vehicles) {
    if (other.id == ego.id) continue;

    const double dx = std::abs(other.state.px - ego.state.px);
    if (dx > 15.0) continue;

    const double dy = other.state.py - ego.state.py;
    const double abs_dy = std::abs(dy);

    if (other.lane_id == ego.lane_id) continue;
    if (abs_dy > avoid_threshold) continue;

    if (dy > 0.0 && cur_lane->left_lane_id > 0 &&
        other.lane_id == static_cast<uint32_t>(cur_lane->left_lane_id)) {
      return LatBehavior::AvoidLeft;
    }
    if (dy < 0.0 && cur_lane->right_lane_id > 0 &&
        other.lane_id == static_cast<uint32_t>(cur_lane->right_lane_id)) {
      return LatBehavior::AvoidRight;
    }
  }

  return LatBehavior::LaneKeeping;
}

BehaviorPlanner::AdaptiveWeights BehaviorPlanner::computeAdaptiveWeights(
    const Vehicle& ego,
    const std::vector<Vehicle>& vehicles,
    const std::vector<Lane>& lanes) const {
  AdaptiveWeights w;

  if (!calibration_.adapt_weights_enable) {
    return w;
  }

  const auto lead = control::FindLeadingVehicle(
      ego, vehicles, lanes, 50.0, calibration_.max_v, 1.0, true);

  double front_dist = (lead.state.px > ego.state.px)
                      ? (lead.state.px - ego.state.px)
                      : 1e6;

  double rel_v = ego.state.speed - lead.state.speed;

  if (front_dist < calibration_.adapt_front_dist_near) {
    w.safe_cost_factor = 2.0;
  } else if (front_dist < calibration_.adapt_front_dist_far) {
    double ratio = (front_dist - calibration_.adapt_front_dist_near) /
                   (calibration_.adapt_front_dist_far - calibration_.adapt_front_dist_near);
    w.safe_cost_factor = 1.0 + (2.0 - 1.0) * (1.0 - ratio);
  } else {
    w.safe_cost_factor = 1.0;
  }

  if (front_dist > calibration_.adapt_front_dist_far && rel_v < calibration_.adapt_front_rel_v_threshold) {
    w.effic_cost_factor = 1.5;
  } else {
    w.effic_cost_factor = 1.0;
  }

  if (front_dist < calibration_.adapt_front_dist_near && rel_v > calibration_.adapt_front_rel_v_threshold) {
    w.action_cost_factor = 2.0;
  } else {
    w.action_cost_factor = 1.0;
  }

  return w;
}

std::pair<LatBehavior, LonBehavior> BehaviorPlanner::parseBehavior(const std::string& behavior) {
  if (behavior == "Keep-MainTain") return {LatBehavior::LaneKeeping, LonBehavior::MainTain};
  if (behavior == "Keep-Accelerate") return {LatBehavior::LaneKeeping, LonBehavior::Accelerate};
  if (behavior == "Keep-Decelerate") return {LatBehavior::LaneKeeping, LonBehavior::Decelerate};
  if (behavior == "Left-MainTain") return {LatBehavior::LaneChangeLeft, LonBehavior::MainTain};
  if (behavior == "Left-Accelerate") return {LatBehavior::LaneChangeLeft, LonBehavior::Accelerate};
  if (behavior == "Left-Decelerate") return {LatBehavior::LaneChangeLeft, LonBehavior::Decelerate};
  if (behavior == "Right-MainTain") return {LatBehavior::LaneChangeRight, LonBehavior::MainTain};
  if (behavior == "Right-Accelerate") return {LatBehavior::LaneChangeRight, LonBehavior::Accelerate};
  if (behavior == "AvdLeft-MainTain") return {LatBehavior::AvoidLeft, LonBehavior::MainTain};
  if (behavior == "AvdLeft-Accelerate") return {LatBehavior::AvoidLeft, LonBehavior::Accelerate};
  if (behavior == "AvdLeft-Decelerate") return {LatBehavior::AvoidLeft, LonBehavior::Decelerate};
  if (behavior == "AvdRight-MainTain") return {LatBehavior::AvoidRight, LonBehavior::MainTain};
  if (behavior == "AvdRight-Accelerate") return {LatBehavior::AvoidRight, LonBehavior::Accelerate};
  if (behavior == "AvdRight-Decelerate") return {LatBehavior::AvoidRight, LonBehavior::Decelerate};
  return {LatBehavior::LaneChangeRight, LonBehavior::Decelerate};
}

std::string BehaviorPlanner::encodeBehavior(LatBehavior lat_behavior, LonBehavior lon_behavior) {
  std::string lat;
  switch (lat_behavior) {
    case LatBehavior::LaneKeeping:     lat = "Keep"; break;
    case LatBehavior::LaneChangeLeft:  lat = "Left"; break;
    case LatBehavior::LaneChangeRight: lat = "Right"; break;
    case LatBehavior::AvoidLeft:       lat = "AvdLeft"; break;
    case LatBehavior::AvoidRight:      lat = "AvdRight"; break;
  }
  std::string lon = "MainTain";
  if (lon_behavior == LonBehavior::Accelerate) {
    lon = "Accelerate";
  } else if (lon_behavior == LonBehavior::Decelerate) {
    lon = "Decelerate";
  }
  return lat + "-" + lon;
}

void BehaviorPlanner::updateTree(TreeNode& node, int layer,
                                 const std::vector<LatBehavior>& lat_behaviors) const {
  static const std::vector<LonBehavior> lon_behaviors = {
      LonBehavior::MainTain, LonBehavior::Accelerate, LonBehavior::Decelerate};

  for (const auto lat : lat_behaviors) {
    for (const auto lon : lon_behaviors) {
      TreeNode child;
      child.type = encodeBehavior(lat, lon);
      child.layer = layer;
      node.children.push_back(child);
    }
  }
}

double BehaviorPlanner::evaluateTraj(const Vehicle& ego,
                                     const Vehicle& ego_before,
                                     LatBehavior lat_behavior,
                                     LonBehavior lon_behavior,
                                     LatBehavior current_lat,
                                     LonBehavior current_lon,
                                     const Trajectory& sim_traj,
                                     const std::vector<Trajectory>& surround_trajs,
                                     const std::vector<Lane>& lanes,
                                     uint32_t target_lane_id,
                                     double target_x,
                                     int layer) {
  double effic = calibration_.effic_cost * adaptive_weights_.effic_cost_factor *
                 (sim_traj.points.back().x - sim_traj.points.front().x);
  const double lane_ref_speed = control::LaneRefSpeedOrDefault(lanes, ego_before.lane_id, calibration_.max_v);
  for (const auto& p : sim_traj.points) {
    if (p.v < lane_ref_speed) {
      effic -= calibration_.effic_cost * adaptive_weights_.effic_cost_factor * (lane_ref_speed - p.v);
    }
  }
  effic = utils::clamp(effic, -100.0, 100.0);

  double safe = 0.0;
  for (const auto& traj : surround_trajs) {
    for (size_t i = 0; i < std::min(sim_traj.points.size(), traj.points.size()); ++i) {
      const double dx = std::abs(sim_traj.points[i].x - traj.points[i].x);
      const double dy = std::abs(sim_traj.points[i].y - traj.points[i].y);
      if (dy < ego.width && dx < ego.length * 1.2 + 2.0) {
        safe -= calibration_.safe_cost * adaptive_weights_.safe_cost_factor / std::max(0.2, dy);
      }
    }
  }
  safe = utils::clamp(safe, -100.0, 0.0);

  double rss = 0.0;
  for (size_t i = 0; i < sim_traj.points.size(); ++i) {
    const auto& ego_p = sim_traj.points[i];
    double min_front_dx = std::numeric_limits<double>::max();
    const TrajectoryPoint* front = nullptr;

    for (const auto& traj : surround_trajs) {
      if (i >= traj.points.size()) {
        continue;
      }
      const auto& other_p = traj.points[i];
      const double dy = std::abs(ego_p.y - other_p.y);
      const double dx = other_p.x - ego_p.x;
      if (dy < ego.width * 1.2 && dx > 0.0 && dx < min_front_dx) {
        min_front_dx = dx;
        front = &other_p;
      }
    }

    if (front) {
      const double safe_gap = control::CalcRssSafeDistance(ego_p.v, front->v, calibration_);
      const double actual_gap = std::max(0.0, min_front_dx - ego.length);
      if (actual_gap < safe_gap) {
        const double violation_ratio = (safe_gap - actual_gap) / std::max(1.0, safe_gap);
        rss -= calibration_.rss_cost * violation_ratio;
      }
    }
  }
  rss = utils::clamp(rss, -100.0, 0.0);

  double action = 0.0;
  const bool is_avoid = (lat_behavior == LatBehavior::AvoidLeft ||
                         lat_behavior == LatBehavior::AvoidRight);
  const bool current_is_avoid = (current_lat == LatBehavior::AvoidLeft ||
                                 current_lat == LatBehavior::AvoidRight);

  if (current_lat != LatBehavior::LaneKeeping && lat_behavior == LatBehavior::LaneKeeping) {
    action -= calibration_.action_cost * adaptive_weights_.action_cost_factor * 5.0;
  } else if (current_lat == LatBehavior::LaneKeeping && lat_behavior != LatBehavior::LaneKeeping) {
    action -= calibration_.action_cost * adaptive_weights_.action_cost_factor * (is_avoid ? 1.0 : 2.0);
  } else if (current_is_avoid && !is_avoid && lat_behavior != LatBehavior::LaneKeeping) {
    action -= calibration_.action_cost * adaptive_weights_.action_cost_factor * 2.0;
  }
  if (lon_behavior == LonBehavior::Accelerate || lon_behavior == LonBehavior::Decelerate) {
    action -= calibration_.action_cost * adaptive_weights_.action_cost_factor;
  }
  if (lon_behavior == current_lon) {
    action += calibration_.action_cost;
  }
  action = utils::clamp(action, -100.0, 0.0);

  double navi = 0.0;
  const double target_dist = target_x - ego_before.state.px;
  if (target_dist > 5.0 && target_dist < 500.0) {
    if (ego_before.lane_id == target_lane_id) {
      navi = (lat_behavior == LatBehavior::LaneKeeping) ? calibration_.navi_cost / 20.0
                                                         : -calibration_.navi_cost / target_dist;
    } else if (ego_before.lane_id > target_lane_id) {
      navi = (lat_behavior == LatBehavior::LaneChangeRight)
                 ? calibration_.navi_cost / target_dist
                 : -calibration_.navi_cost / target_dist;
    } else {
      navi = (lat_behavior == LatBehavior::LaneChangeLeft)
                 ? calibration_.navi_cost / target_dist
                 : -calibration_.navi_cost / target_dist;
    }
  }
  navi = utils::clamp(navi, -100.0, 100.0);

  double affect = 0.0;
  for (const auto& traj : surround_trajs) {
    for (const auto& p : traj.points) {
      if (p.a < 0.0) {
        affect += calibration_.affect_cost * p.a;
      }
    }
  }
  affect = utils::clamp(affect, -200.0, 0.0);

  const double score =
      (200.0 + effic + safe + rss + action + navi + affect) * std::pow(calibration_.fade_factor, layer);
  return score;
}

void BehaviorPlanner::mctsOnce(TreeNode& node,
                               int iter,
                               std::mt19937& rng,
                               const std::vector<Lane>& lanes,
                               uint32_t target_lane_id,
                               double target_x,
                               int layer) {
  (void)rng;
  if (node.layer >= calibration_.tree_hight) {
    node.n += 1.0;
    return;
  }

  if (node.children.empty()) {
    const auto candidates = getPotentialBehaviors(node.vehicles.front(), node.vehicles, lanes, target_lane_id);
    updateTree(node, layer + 1, candidates);
  }

  if (node.children.empty()) {
    node.n += 1.0;
    return;
  }

  size_t best = 0;
  double best_ucb = -1e18;
  for (size_t i = 0; i < node.children.size(); ++i) {
    auto& child = node.children[i];
    child.ucb = child.t / child.n + calibration_.search_new_wight * std::sqrt(std::log(iter + 1.0) / child.n);
    if (child.ucb > best_ucb) {
      best_ucb = child.ucb;
      best = i;
    }
  }

  auto& child = node.children[best];
  const auto [lat, lon] = parseBehavior(child.type);
  const LatBehavior current_lat = (layer == 0) ? node.vehicles.front().lat_behavior
                                                : parseBehavior(node.type).first;
  const LonBehavior current_lon = (layer == 0) ? node.vehicles.front().lon_behavior
                                                : parseBehavior(node.type).second;

  if (child.n > 1.0) {
    const double old_t = child.t;
    if (child.vehicles.empty()) {
      child.vehicles = node.vehicles;
    }
    mctsOnce(child, iter, rng, lanes, target_lane_id, target_x, layer + 1);
    node.t += (child.t - old_t);
  } else {
    const SimResult sim = forwardSimulate(node.vehicles, lat, lon, lanes, layer + 1);
    if (!sim.success) {
      if (calibration_.mcts_delete_on_failure) {
        node.children.erase(node.children.begin() + best);
      } else {
        child.n += 1.0;
        child.v = -1000.0;
        child.t = -1000.0;
        node.t += child.t;
      }
      node.n += 1.0;
      return;
    }
    const double score = evaluateTraj(node.vehicles.front(), node.vehicles.front(), lat, lon,
                                      current_lat, current_lon, sim.ego, sim.agents,
                                      lanes, target_lane_id, target_x, layer);
    child.n += 1.0;
    child.v = score;
    child.t = score;
    child.vehicles = sim.end_vehicles;
    node.t += child.t;
  }

  node.n += 1.0;
}

SearchResult BehaviorPlanner::getResult(const TreeNode& root,
                                        const std::vector<Vehicle>& vehicles,
                                        const std::vector<Lane>& lanes,
                                        uint32_t target_lane_id,
                                        double target_x) {
  SearchResult result;
  TreeNode current = root;
  std::vector<std::string> decisions;

  while (!current.children.empty()) {
    const auto best_it = std::max_element(current.children.begin(), current.children.end(),
                                          [](const TreeNode& a, const TreeNode& b) {
                                            return a.t < b.t;
                                          });
    if (best_it == current.children.end()) {
      break;
    }
    decisions.push_back(best_it->type);
    current = *best_it;
  }

  if (!decisions.empty()) {
    result.decision_type = decisions.front();
    const auto [lat, lon] = parseBehavior(decisions.front());
    result.lat_behavior = lat;
    result.lon_behavior = lon;
  }
  result.value = current.t;
  result.score = current.v;

  std::vector<Vehicle> sim_vehicles = vehicles;
  result.coarse_ego.points.push_back(utils::fromState(sim_vehicles[0], 0.0));
  result.coarse_agents.assign(sim_vehicles.size() - 1, Trajectory{});
  for (size_t k = 1; k < sim_vehicles.size(); ++k) {
    result.coarse_agents[k - 1].points.push_back(utils::fromState(sim_vehicles[k], 0.0));
  }

  for (size_t i = 0; i < decisions.size(); ++i) {
    const auto [lat, lon] = parseBehavior(decisions[i]);
    const auto sim = forwardSimulate(sim_vehicles, lat, lon, lanes, static_cast<int>(i + 1));
    if (!sim.success) {
      break;
    }
    sim_vehicles = sim.end_vehicles;
    result.coarse_ego.points.insert(result.coarse_ego.points.end(),
                                    sim.ego.points.begin(), sim.ego.points.end());
    for (size_t k = 0; k < sim.agents.size(); ++k) {
      result.coarse_agents[k].points.insert(result.coarse_agents[k].points.end(),
                                            sim.agents[k].points.begin(),
                                            sim.agents[k].points.end());
    }
  }

  if (result.coarse_ego.points.size() < 2) {
    result.coarse_ego.points.push_back(utils::fromState(vehicles[0], calibration_.sim_ts));
  }

  (void)target_lane_id;
  (void)target_x;
  return result;
}

SearchResult BehaviorPlanner::plan(const std::vector<Vehicle>& vehicles,
                                   const std::vector<Lane>& lanes,
                                   int cycle,
                                   uint32_t target_lane_id,
                                   double target_x) {
  SearchResult result;
  if (vehicles.empty() || lanes.empty()) {
    return result;
  }

  adaptive_weights_ = computeAdaptiveWeights(vehicles.front(), vehicles, lanes);

  TreeNode root;
  root.type = "Root";
  root.layer = 0;
  root.vehicles = vehicles;

  std::mt19937 rng(42 + cycle);
  const int iter_num = std::max(1, static_cast<int>(std::round(calibration_.max_search_times * 0.1)));

  const auto mcts_t0 = std::chrono::steady_clock::now();
  for (int iter = 1; iter <= iter_num; ++iter) {
    if (root.children.empty()) {
      updateTree(root, 1, getPotentialBehaviors(vehicles.front(), vehicles, lanes, target_lane_id));
    }
    if (root.children.empty()) {
      break;
    }
    mctsOnce(root, iter, rng, lanes, target_lane_id, target_x, 0);
  }
  const auto mcts_t1 = std::chrono::steady_clock::now();
  const double mcts_ms = std::chrono::duration<double, std::milli>(mcts_t1 - mcts_t0).count();
  RCLCPP_INFO(getLogger(), "MCTS %d iters done in %.2f ms", iter_num, mcts_ms);

  return getResult(root, vehicles, lanes, target_lane_id, target_x);
}

}  // namespace lcs
