#include "lane_change_sim/planner_core.hpp"

namespace lcs {

PlannerCore::PlannerCore() {
  behavior_planner_.setCalibration(calibration_);
  trajectory_planner_.setCalibration(calibration_);
}

void PlannerCore::setCalibration(const Calibration& calibration) {
  calibration_ = calibration;
  behavior_planner_.setCalibration(calibration_);
  trajectory_planner_.setCalibration(calibration_);
}

SearchResult PlannerCore::plan(const std::vector<Vehicle>& vehicles,
                               const std::vector<Lane>& lanes,
                               int cycle,
                               uint32_t target_lane_id,
                               double target_x) {
  SearchResult result = behavior_planner_.plan(vehicles, lanes, cycle, target_lane_id, target_x);

  result.optimized_ego = trajectory_planner_.plan(
      result.coarse_ego,
      result.coarse_agents,
      vehicles,
      lanes);

  return result;
}

}  // namespace lcs
