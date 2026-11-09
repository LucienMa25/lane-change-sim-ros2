#pragma once

#include <vector>

#include "lane_change_sim/types.hpp"
#include "lane_change_sim/behavior_planner.hpp"
#include "lane_change_sim/trajectory_planner.hpp"

namespace lcs {

class PlannerCore {
 public:
  PlannerCore();

  void setCalibration(const Calibration& calibration);

  SearchResult plan(const std::vector<Vehicle>& vehicles,
                    const std::vector<Lane>& lanes,
                    int cycle,
                    uint32_t target_lane_id,
                    double target_x);

  const std::vector<DriveCube>& getLastCubes() const { return trajectory_planner_.getLastCubes(); }

 private:
    Calibration calibration_;
    BehaviorPlanner behavior_planner_;
    TrajectoryPlanner trajectory_planner_;
};

}  // namespace lcs
