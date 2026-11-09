#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "lane_change_sim/control/common.hpp"
#include "lane_change_sim/types.hpp"
#include "lane_change_sim/utils/math_utils.hpp"

namespace lcs::control {

class LaneChangeStateManager {
 public:
  virtual ~LaneChangeStateManager() = default;

 protected:
  uint32_t last_ego_lane_id_{0};

  static bool stage2Complete(const Vehicle& ego, const std::vector<Lane>& lanes) {
    const Lane* cur_l = FindLaneById(lanes, ego.lane_id);
    const double lane_y = cur_l ? InterpLaneY(ego.state.px, *cur_l) : ego.state.py;
    return std::abs(ego.state.py - lane_y) < 0.25 &&
           std::abs(utils::normalizeAngle(ego.state.heading)) < 0.1;
  }

  static bool abortingComplete(const Vehicle& ego, const std::vector<Lane>& lanes) {
    const Lane* cur_l = FindLaneById(lanes, ego.lane_id);
    const double lane_y = cur_l ? InterpLaneY(ego.state.px, *cur_l) : ego.state.py;
    return std::abs(ego.state.py - lane_y) < 0.5 &&
           std::abs(utils::normalizeAngle(ego.state.heading)) < 0.2;
  }

  static void applyLockedBehavior(Vehicle& ego) {
    if (ego.lane_change_stage == LaneChangeStage::Stage2 ||
        ego.lane_change_stage == LaneChangeStage::Aborting) {
      ego.lat_behavior = LatBehavior::LaneKeeping;
    }
  }

  void updateLastEgoLaneId(const Vehicle& ego) {
    last_ego_lane_id_ = ego.lane_id;
  }
};

class StateManagerForSim : public LaneChangeStateManager {
 public:
  void reset() {
    lane_change_finish_ = false;
  }

  void update(Vehicle& ego, LatBehavior lat_behavior,
              LonBehavior lon_behavior,
              const std::vector<Lane>& lanes,
              int layer, int step) {
    if (layer == 1 && step == 1) {
      last_ego_lane_id_ = ego.lane_id;
    }

    if (lane_change_finish_) {
      lat_behavior = LatBehavior::LaneKeeping;
    }

    ego.lon_behavior = lon_behavior;

    if (lat_behavior == LatBehavior::AvoidLeft ||
        lat_behavior == LatBehavior::AvoidRight) {
      ego.lat_behavior = lat_behavior;
      if (ego.lane_change_stage == LaneChangeStage::Stage0 ||
          ego.lane_change_stage == LaneChangeStage::Stage1) {
        ego.lane_change_stage = LaneChangeStage::Aborting;
      }
      updateLastEgoLaneId(ego);
      return;
    }

    switch (ego.lane_change_stage) {
      case LaneChangeStage::Off:
        if (lat_behavior == LatBehavior::LaneKeeping) {
          ego.lat_behavior = lat_behavior;
        } else {
          ego.lat_behavior = lat_behavior;
          ego.lane_change_stage = LaneChangeStage::Stage0;
        }
        break;

      case LaneChangeStage::Stage0:
        if (lat_behavior == LatBehavior::LaneKeeping) {
          ego.lane_change_stage = LaneChangeStage::Aborting;
        } else {
          ego.lane_change_stage = LaneChangeStage::Stage1;
        }
        break;

      case LaneChangeStage::Stage1:
        if (lat_behavior == LatBehavior::LaneKeeping) {
          ego.lane_change_stage = LaneChangeStage::Aborting;
        } else {
          if (ego.lane_id != last_ego_lane_id_) {
            ego.lane_change_stage = LaneChangeStage::Stage2;
          }
        }
        break;

      case LaneChangeStage::Stage2:
        if (stage2Complete(ego, lanes)) {
          ego.lane_change_stage = LaneChangeStage::Off;
          lane_change_finish_ = true;
        }
        break;

      case LaneChangeStage::Aborting:
        if (abortingComplete(ego, lanes)) {
          ego.lane_change_stage = LaneChangeStage::Off;
          lane_change_finish_ = true;
        }
        break;
    }

    applyLockedBehavior(ego);
    updateLastEgoLaneId(ego);
  }

 private:
  bool lane_change_finish_{false};
};

class StateManagerRealtime : public LaneChangeStateManager {
 public:
  void update(Vehicle& ego, LatBehavior lat_behavior,
              LonBehavior lon_behavior,
              std::vector<Vehicle>& vehicles,
              const std::vector<Lane>& lanes,
              double ts) {
    for (auto& v : vehicles) v.intention = Intention::None;
    const Intention intention = getIntention(ego, lat_behavior, vehicles, lanes);

    if (ego.lane_change_stage == LaneChangeStage::Stage0 ||
        ego.lane_change_stage == LaneChangeStage::Stage1) {
      uint32_t target_lane_id = 0;
      const Lane* cur_l = FindLaneById(lanes, ego.lane_id);
      if (cur_l) {
        if (lat_behavior == LatBehavior::LaneChangeLeft)
          target_lane_id = static_cast<uint32_t>(cur_l->left_lane_id);
        else if (lat_behavior == LatBehavior::LaneChangeRight)
          target_lane_id = static_cast<uint32_t>(cur_l->right_lane_id);
      }
      if (target_lane_id > 0) {
        Vehicle* key_v = nullptr;
        double best_dist = std::numeric_limits<double>::max();
        for (auto& v : vehicles) {
          if (v.id == ego.id || v.lane_id != target_lane_id || v.state.px >= ego.state.px) continue;
          const double dx = ego.state.px - v.state.px;
          if (dx < best_dist) { best_dist = dx; key_v = &v; }
        }
        if (key_v) key_v->intention = intention;
      }
    }

    ego.lon_behavior = lon_behavior;

    if (lat_behavior == LatBehavior::AvoidLeft ||
        lat_behavior == LatBehavior::AvoidRight) {
      ego.lat_behavior = lat_behavior;
      if (ego.lane_change_stage == LaneChangeStage::Stage0 ||
          ego.lane_change_stage == LaneChangeStage::Stage1) {
        ego.lane_change_stage = LaneChangeStage::Aborting;
      }
      updateLastEgoLaneId(ego);
      return;
    }

    switch (ego.lane_change_stage) {
      case LaneChangeStage::Off:
        lc_finish_time_ += ts;
        if (lat_behavior == LatBehavior::LaneKeeping) {
          ego.lat_behavior = lat_behavior;
        } else if (lc_finish_time_ > 1.0) {
          ego.lat_behavior = lat_behavior;
          ego.lane_change_stage = LaneChangeStage::Stage0;
          lc_finish_time_ = 0.0;
          game_time_ = 0.0;
        } else {
          ego.lat_behavior = lat_behavior;
        }
        break;

      case LaneChangeStage::Stage0:
        if (lat_behavior == LatBehavior::LaneKeeping) {
          ego.lane_change_stage = LaneChangeStage::Aborting;
          game_time_ = 0.0;
        } else {
          if (intention == Intention::Yield) {
            ego.lane_change_stage = LaneChangeStage::Stage1;
            game_time_ = 0.0;
          } else if (intention == Intention::Aggresive) {
            const Lane* cur_l = FindLaneById(lanes, ego.lane_id);
            const double lane_y = cur_l ? InterpLaneY(ego.state.px, *cur_l) : ego.state.py;
            if (std::abs(ego.state.py - lane_y) > 0.8) {
              if (game_time_ > 2.0) {
                game_time_ = 0.0;
                ego.lane_change_stage = LaneChangeStage::Aborting;
              } else {
                game_time_ += ts;
              }
            } else {
              game_time_ = 0.0;
            }
          } else {
            game_time_ = 0.0;
          }
        }
        break;

      case LaneChangeStage::Stage1:
        if (lat_behavior == LatBehavior::LaneKeeping) {
          ego.lane_change_stage = LaneChangeStage::Aborting;
        } else {
          if (intention == Intention::Yield) {
            if (ego.lane_id != last_ego_lane_id_ && last_ego_lane_id_ > 0) {
              ego.lane_change_stage = LaneChangeStage::Stage2;
            }
          } else if (intention == Intention::None) {
            ego.lane_change_stage = LaneChangeStage::Stage0;
          } else {
            ego.lane_change_stage = LaneChangeStage::Aborting;
          }
        }
        break;

      case LaneChangeStage::Stage2:
        if (stage2Complete(ego, lanes)) {
          ego.lane_change_stage = LaneChangeStage::Off;
          ego.lat_behavior = LatBehavior::LaneKeeping;
          lc_finish_time_ = 0.0;
        } else {
          ego.lat_behavior = LatBehavior::LaneKeeping;
        }
        break;

      case LaneChangeStage::Aborting:
        if (abortingComplete(ego, lanes)) {
          ego.lane_change_stage = LaneChangeStage::Off;
          ego.lat_behavior = LatBehavior::LaneKeeping;
          lc_finish_time_ = 0.0;
        } else {
          ego.lat_behavior = LatBehavior::LaneKeeping;
        }
        break;
    }

    updateLastEgoLaneId(ego);
  }

  static Intention getIntention(const Vehicle& ego,
                                LatBehavior lat_behavior,
                                const std::vector<Vehicle>& vehicles,
                                const std::vector<Lane>& lanes) {
    if (ego.lane_change_stage != LaneChangeStage::Stage0 &&
        ego.lane_change_stage != LaneChangeStage::Stage1 &&
        ego.lane_change_stage != LaneChangeStage::Aborting) {
      return Intention::None;
    }

    uint32_t target_lane_id = 0;
    const Lane* cur_l = FindLaneById(lanes, ego.lane_id);
    if (!cur_l) return Intention::None;

    if (lat_behavior == LatBehavior::LaneChangeLeft) {
      target_lane_id = static_cast<uint32_t>(cur_l->left_lane_id);
    } else if (lat_behavior == LatBehavior::LaneChangeRight) {
      target_lane_id = static_cast<uint32_t>(cur_l->right_lane_id);
    }
    if (target_lane_id == 0) return Intention::None;

    const Vehicle* key_vehicle = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    for (const auto& v : vehicles) {
      if (v.id == ego.id) continue;
      if (v.lane_id != target_lane_id) continue;
      if (v.state.px >= ego.state.px) continue;
      const double dx = ego.state.px - v.state.px;
      if (dx < best_dist) {
        best_dist = dx;
        key_vehicle = &v;
      }
    }

    if (!key_vehicle) {
      return Intention::Yield;
    }

    const double gap_time =
        (ego.state.px - key_vehicle->state.px - ego.length) /
        std::max(0.1, key_vehicle->state.speed);

    const double yield_acc = gap_time - 1.0;
    const double aggr_acc  = gap_time - 0.5;

    if (gap_time < 0.5) {
      return Intention::Aggresive;
    } else if (gap_time > 1.0) {
      return Intention::Yield;
    } else {
      if (key_vehicle->acc < yield_acc) {
        return Intention::Yield;
      } else if (key_vehicle->acc > aggr_acc) {
        return Intention::Aggresive;
      } else {
        return Intention::None;
      }
    }
  }

 private:
  double lc_finish_time_{0.0};
  double game_time_{0.0};
};

}  // namespace lcs::control
