#pragma once

#include <vector>

#ifdef HAS_CASADI
#include <casadi/casadi.hpp>
#endif

#include "lane_change_sim/types.hpp"

namespace lcs {

class TrajectoryPlanner {
 public:
  TrajectoryPlanner();
  void setCalibration(const Calibration& calibration);

  Trajectory plan(const Trajectory& coarse_ego,
                  const std::vector<Trajectory>& coarse_agents,
                  const std::vector<Vehicle>& vehicles,
                  const std::vector<Lane>& lanes) const;

  const std::vector<DriveCube>& getLastCubes() const { return last_cubes_; }

 private:
  Calibration calibration_;
  mutable std::vector<DriveCube> last_cubes_;

#ifdef HAS_CASADI
  mutable casadi::Function solver_;
  mutable int cached_n_{0};
  mutable Trajectory prev_optimal_;
  mutable double prev_ego_px_{0.0};
#endif

  std::vector<DriveCube> generateDriveCubes(const Trajectory& ego_traj,
                                            const std::vector<Trajectory>& agent_trajs,
                                            const std::vector<Vehicle>& vehicles,
                                            const std::vector<Lane>& lanes) const;

  Trajectory initGuess(const Trajectory& coarse, const Vehicle& ego) const;
  Trajectory optimizeTrajectory(const Trajectory& ref,
                                const std::vector<DriveCube>& cubes) const;
};

}  // namespace lcs
