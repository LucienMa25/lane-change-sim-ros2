#pragma once

#include <random>
#include <string>
#include <utility>
#include <vector>

#include "lane_change_sim/control/state_manager.hpp"
#include "lane_change_sim/types.hpp"

namespace lcs {

class BehaviorPlanner {
 public:
  BehaviorPlanner();
  void setCalibration(const Calibration& calibration);

  SearchResult plan(const std::vector<Vehicle>& vehicles,
                    const std::vector<Lane>& lanes,
                    int cycle,
                    uint32_t target_lane_id,
                    double target_x);

 private:
  struct TreeNode {
    std::string type{"Root"};
    int layer{0};
    std::vector<Vehicle> vehicles;
    double n{1e-6};
    double v{0.0};
    double t{0.0};
    double ucb{0.0};
    std::vector<TreeNode> children;
  };

  struct SimResult {
    bool success{false};
    Trajectory ego;
    std::vector<Trajectory> agents;
    std::vector<Vehicle> end_vehicles;
  };

  struct AdaptiveWeights {
    double safe_cost_factor{1.0};
    double effic_cost_factor{1.0};
    double action_cost_factor{1.0};
  };

  Calibration calibration_;
  mutable AdaptiveWeights adaptive_weights_;
  control::StateManagerForSim state_mgr_for_sim_;

  double evaluateMobilGain(const Vehicle& ego,
                           const std::vector<Vehicle>& vehicles,
                           const std::vector<Lane>& lanes,
                           uint32_t target_lane_id) const;

  LatBehavior checkAvoidNeeded(const Vehicle& ego,
                               const std::vector<Vehicle>& vehicles,
                               const std::vector<Lane>& lanes) const;

  std::vector<LatBehavior> getPotentialBehaviors(const Vehicle& ego,
                                                 const std::vector<Vehicle>& vehicles,
                                                 const std::vector<Lane>& lanes,
                                                 uint32_t target_lane_id = 0) const;

  AdaptiveWeights computeAdaptiveWeights(const Vehicle& ego,
                                         const std::vector<Vehicle>& vehicles,
                                         const std::vector<Lane>& lanes) const;

  static std::pair<LatBehavior, LonBehavior> parseBehavior(const std::string& behavior);
  static std::string encodeBehavior(LatBehavior lat_behavior, LonBehavior lon_behavior);

  void updateTree(TreeNode& node, int layer,
                  const std::vector<LatBehavior>& lat_behaviors) const;

  void propagateOnce(Vehicle& vehicle, const Vehicle& leading,
                     const std::vector<Lane>& lanes) const;

  /// Lane-change-aware propagation: blends current-lane IDM with target-gap tracking.
  /// Inspired by EPSILON's PropagateOnceAdvancedLC + GetTargetStateOnTargetLane.
  void propagateOnceLC(Vehicle& vehicle,
                       const Vehicle& current_leading,
                       const Vehicle* gap_front,
                       const Vehicle* gap_rear,
                       uint32_t target_lane_id,
                       const std::vector<Lane>& lanes) const;

  SimResult forwardSimulate(const std::vector<Vehicle>& vehicles,
                            LatBehavior lat_behavior,
                            LonBehavior lon_behavior,
                            const std::vector<Lane>& lanes,
                            int layer);

  double evaluateTraj(const Vehicle& ego,
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
                      int layer);

  void mctsOnce(TreeNode& node,
                int iter,
                std::mt19937& rng,
                const std::vector<Lane>& lanes,
                uint32_t target_lane_id,
                double target_x,
                int layer);

  SearchResult getResult(const TreeNode& root,
                         const std::vector<Vehicle>& vehicles,
                         const std::vector<Lane>& lanes,
                         uint32_t target_lane_id,
                         double target_x);
};

}  // namespace lcs
