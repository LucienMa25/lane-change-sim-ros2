#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lane_change_sim/types.hpp"

namespace lcs {

struct ScenarioConfig {
  std::string mode{"builtin_highway"};
  std::string epsilon_root;
  double lane_width{4.0};
  double ref_speed{7.0};
  std::size_t max_vehicles{7};
  uint32_t target_lane_id{2};
  double target_x{200.0};
};

class ScenarioLoader {
 public:
  bool load(const ScenarioConfig& config,
            std::vector<Lane>* lanes,
            std::vector<Vehicle>* vehicles,
            std::string* scenario_source) const;

 private:
  bool loadBuiltinHighway(const ScenarioConfig& config,
                          std::vector<Lane>* lanes,
                          std::vector<Vehicle>* vehicles,
                          std::string* scenario_source) const;

  bool loadEpsilonHighway(const ScenarioConfig& config,
                          std::vector<Lane>* lanes,
                          std::vector<Vehicle>* vehicles,
                          std::string* scenario_source) const;
};

}  // namespace lcs
