#include "lane_change_sim/trajectory_planner.hpp"
#include "lane_change_sim/utils/collision_utils.hpp"
#include "lane_change_sim/utils/math_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#ifdef HAS_CASADI
#include <casadi/casadi.hpp>
#endif

namespace lcs {

namespace {

auto getLogger() { return rclcpp::get_logger("TrajectoryPlanner"); }

bool collidesAtCorner(double ego_x, double ego_y,
                      double ego_heading,
                      const TrajectoryPoint& agent_pt,
                      const Vehicle& ego_vehicle,
                      const Vehicle& agent_vehicle,
                      double inflate_factor) {
  const utils::Box2d ego_box = utils::makeBox(ego_x, ego_y, ego_heading,
                                              ego_vehicle.length, ego_vehicle.width,
                                              inflate_factor);
  const utils::Box2d agent_box = utils::makeBox(agent_pt.x, agent_pt.y, agent_pt.h,
                                                agent_vehicle.length, agent_vehicle.width,
                                                inflate_factor);
  return ego_box.hasOverlap(agent_box);
}

struct TestPoint {
  double x, y, heading;
  int agent_idx;
};

bool anyCollision(const std::vector<TestPoint>& pts,
                  int i,
                  const std::vector<Trajectory>& agent_trajs,
                  const std::vector<Vehicle>& vehicles,
                  const Vehicle& ego_vehicle,
                  double inflate_factor) {
  for (size_t k = 0; k < agent_trajs.size() && k + 1 < vehicles.size(); ++k) {
    bool agent_ok = true;
    for (const auto& pt : pts) {
      if (pt.agent_idx >= static_cast<int>(agent_trajs[k].points.size())) {
        agent_ok = false;
        break;
      }
    }
    if (!agent_ok) continue;
    for (const auto& pt : pts) {
      if (collidesAtCorner(pt.x, pt.y, pt.heading,
                           agent_trajs[k].points[pt.agent_idx],
                           ego_vehicle, vehicles[k + 1], inflate_factor)) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

TrajectoryPlanner::TrajectoryPlanner() = default;

void TrajectoryPlanner::setCalibration(const Calibration& calibration) {
  calibration_ = calibration;
}

std::vector<DriveCube> TrajectoryPlanner::generateDriveCubes(const Trajectory& ego_traj,
                                                             const std::vector<Trajectory>& agent_trajs,
                                                             const std::vector<Vehicle>& vehicles,
                                                             const std::vector<Lane>& lanes) const {
  std::vector<DriveCube> cubes;
  const int n = static_cast<int>(ego_traj.points.size());
  const int cube_num = n - 1;
  if (cube_num < 1) {
    return cubes;
  }
  cubes.resize(cube_num);

  const Vehicle& ego_vehicle = vehicles.front();
  const double inflate_factor = std::min(1.3, std::max(1.0, 0.7 + 0.03 * ego_vehicle.state.speed));
  const int max_inflate_step = calibration_.cube_inflate_steps.empty()
                                   ? 0
                                   : *std::max_element(calibration_.cube_inflate_steps.begin(),
                                                       calibration_.cube_inflate_steps.end());
  const double left_edge = lanes.back().left_y - ego_vehicle.width / 2.0;
  const double right_edge = lanes.front().right_y + ego_vehicle.width / 2.0;

  const double agent_ratio = calibration_.traj_ts / calibration_.sim_ts;

  for (int i = 0; i < cube_num; ++i) {
    const int prev = std::max(0, i - 1);
    const int next = std::min(n - 1, i + 1);
    const auto& pp = ego_traj.points[prev];
    const auto& pn = ego_traj.points[next];

    cubes[i].lb.x = std::min(pp.x, pn.x);
    cubes[i].ub.x = std::max(pp.x, pn.x);
    cubes[i].lb.y = std::min(pp.y, pn.y);
    cubes[i].ub.y = std::max(pp.y, pn.y);
    cubes[i].lb.t = ego_traj.points[i].t;
    cubes[i].ub.t = ego_traj.points[i + 1].t;

    const int agent_i = static_cast<int>(std::round(i * agent_ratio));

    bool x_p_finish = false;
    bool x_n_finish = false;
    bool y_p_finish = false;
    bool y_n_finish = false;

    for (int step = 1; step <= max_inflate_step; ++step) {
      if (!x_p_finish) {
        if (step > calibration_.cube_inflate_steps[0]) {
          x_p_finish = true;
        } else {
          cubes[i].ub.x += calibration_.cube_inflate_dx;
          std::vector<TestPoint> pts = {
            {cubes[i].ub.x, cubes[i].ub.y, ego_traj.points[next].h, agent_i},
            {cubes[i].ub.x, cubes[i].lb.y, ego_traj.points[i].h,   agent_i}
          };
          if (anyCollision(pts, i, agent_trajs, vehicles, ego_vehicle, inflate_factor)) {
            cubes[i].ub.x -= calibration_.cube_inflate_dx;
            x_p_finish = true;
          }
        }
      }

      if (!x_n_finish) {
        if (step > calibration_.cube_inflate_steps[1]) {
          x_n_finish = true;
        } else {
          cubes[i].lb.x -= calibration_.cube_inflate_dx;
          std::vector<TestPoint> pts = {
            {cubes[i].lb.x, cubes[i].ub.y, ego_traj.points[i].h,   agent_i},
            {cubes[i].lb.x, cubes[i].lb.y, ego_traj.points[prev].h, agent_i}
          };
          if (anyCollision(pts, i, agent_trajs, vehicles, ego_vehicle, inflate_factor)) {
            cubes[i].lb.x += calibration_.cube_inflate_dx;
            x_n_finish = true;
          }
        }
      }

      if (!y_p_finish) {
        if (step > calibration_.cube_inflate_steps[2]) {
          y_p_finish = true;
        } else {
          cubes[i].ub.y += calibration_.cube_inflate_dy;
          std::vector<TestPoint> pts = {
            {cubes[i].ub.x, cubes[i].ub.y, ego_traj.points[next].h, agent_i},
            {cubes[i].lb.x, cubes[i].ub.y, ego_traj.points[i].h,   agent_i}
          };
          if (anyCollision(pts, i, agent_trajs, vehicles, ego_vehicle, inflate_factor) ||
              cubes[i].ub.y > left_edge) {
            cubes[i].ub.y -= calibration_.cube_inflate_dy;
            y_p_finish = true;
          }
        }
      }

      if (!y_n_finish) {
        if (step > calibration_.cube_inflate_steps[3]) {
          y_n_finish = true;
        } else {
          cubes[i].lb.y -= calibration_.cube_inflate_dy;
          std::vector<TestPoint> pts = {
            {cubes[i].ub.x, cubes[i].lb.y, ego_traj.points[next].h, agent_i},
            {cubes[i].lb.x, cubes[i].lb.y, ego_traj.points[i].h,   agent_i}
          };
          if (anyCollision(pts, i, agent_trajs, vehicles, ego_vehicle, inflate_factor) ||
              cubes[i].lb.y < right_edge) {
            cubes[i].lb.y += calibration_.cube_inflate_dy;
            y_n_finish = true;
          }
        }
      }
    }
  }

  return cubes;
}

Trajectory TrajectoryPlanner::initGuess(const Trajectory& coarse, const Vehicle& ego) const {
  Trajectory guess;
  const int traj_size = static_cast<int>(calibration_.traj_time / calibration_.traj_ts) + 1;
  guess.points.resize(traj_size);

  const int interp_num = static_cast<int>(calibration_.sim_ts / calibration_.traj_ts);
  int idx = 0;
  for (size_t i = 0; i + 1 < coarse.points.size() && idx < traj_size; ++i) {
    const auto& a = coarse.points[i];
    const auto& b = coarse.points[i + 1];
    for (int j = 0; j <= interp_num && idx < traj_size; ++j) {
      const double r = static_cast<double>(j) / std::max(1, interp_num);
      TrajectoryPoint p;
      p.x = a.x + r * (b.x - a.x);
      p.y = a.y + r * (b.y - a.y);
      p.v = a.v + r * (b.v - a.v);
      p.h = utils::normalizeAngle(a.h + r * utils::normalizeAngle(b.h - a.h));
      p.a = a.a + r * (b.a - a.a);
      p.t = a.t + r * (b.t - a.t);
      guess.points[idx++] = p;
    }
  }

  while (idx < traj_size) {
    const auto fallback = guess.points[std::max(0, idx - 1)];
    guess.points[idx] = fallback;
    guess.points[idx].t = idx * calibration_.traj_ts;
    ++idx;
  }

  guess.points[0].h = ego.state.heading;
  return guess;
}

Trajectory TrajectoryPlanner::optimizeTrajectory(const Trajectory& ref,
                                                 const std::vector<DriveCube>& cubes) const {
  Trajectory out = ref;

#ifdef HAS_CASADI
  try {
    using namespace casadi;

    const int n = static_cast<int>(ref.points.size());
    const double dt = calibration_.traj_ts;

    if (cached_n_ != n) {
      RCLCPP_INFO(getLogger(), "Creating IPOPT solver for n=%d points", n);

      SX x = SX::sym("x", n);
      SX y = SX::sym("y", n);
      SX theta = SX::sym("theta", n);
      SX v = SX::sym("v", n);
      SX delta = SX::sym("delta", n);
      SX a = SX::sym("a", n);
      SX omega = SX::sym("omega", n);
      SX jerk = SX::sym("jerk", n);

      SX p_inf_w = SX::sym("inf_w");
      SX p_ref_x = SX::sym("ref_x", n, 1);
      SX p_ref_y = SX::sym("ref_y", n, 1);
      SX p_ref_theta = SX::sym("ref_theta", n, 1);
      SX p_cube_center_x = SX::sym("cube_center_x", n, 1);
      SX p_cube_center_y = SX::sym("cube_center_y", n, 1);

      auto prev = Slice(0, n - 1);
      auto next = Slice(1, n);

      SX g_x = x(next) - (x(prev) + dt * v(prev) * cos(theta(prev)));
      SX g_y = y(next) - (y(prev) + dt * v(prev) * sin(theta(prev)));
      SX g_theta = theta(next) - (theta(prev) + dt * v(prev) * tan(delta(prev)) / calibration_.wheel_base);
      SX g_v = v(next) - (v(prev) + dt * a(prev));
      SX g_delta = delta(next) - (delta(prev) + dt * omega(prev));
      SX g_a = a(next) - (a(prev) + dt * jerk(prev));

      SX infeasibility = sumsqr(SX::vertcat({g_x, g_y, g_theta, g_v, g_delta, g_a}));

      SX f_obj =
          calibration_.weight_x * sumsqr(x - p_ref_x) +
          calibration_.weight_y * sumsqr(y - p_ref_y) +
          calibration_.weight_theta * sumsqr(theta - p_ref_theta) +
        calibration_.weight_cube_center_x * sumsqr(x - p_cube_center_x) +
        calibration_.weight_cube_center_y * sumsqr(y - p_cube_center_y) +
          calibration_.weight_omega * sumsqr(omega) +
          calibration_.weight_jerk * sumsqr(jerk) +
          p_inf_w * infeasibility;

      SX opt_x = SX::vertcat({x, y, theta, v, delta, a, omega, jerk});
      SX p = SX::vertcat({p_inf_w, p_ref_x, p_ref_y, p_ref_theta,
                p_cube_center_x, p_cube_center_y});

      SXDict nlp;
      nlp["x"] = opt_x;
      nlp["p"] = p;
      nlp["f"] = f_obj;

      Dict opts;
      Dict ipopt;
  #ifdef WITH_HSL
      ipopt["linear_solver"] = "ma27";
  #else
      ipopt["linear_solver"] = "mumps";
  #endif
      ipopt["print_level"]              = 0;
      ipopt["max_iter"]                 = calibration_.ipopt_max_iter;
      ipopt["tol"]                      = calibration_.ipopt_tol;
      ipopt["acceptable_tol"]           = calibration_.ipopt_acceptable_tol;
      ipopt["acceptable_iter"]          = calibration_.ipopt_acceptable_iter;
      ipopt["acceptable_obj_change_tol"]= 1e-5;
      ipopt["mu_strategy"]              = calibration_.ipopt_mu_strategy;
      opts["ipopt"]    = ipopt;
      opts["print_time"] = 0;

      solver_ = nlpsol("iterative_solver", "ipopt", nlp, opts);
      cached_n_ = n;
    }

    DM ones = DM::ones(n, 1);
    DM lb_x = -inf * ones;
    DM ub_x = inf * ones;
    DM lb_y = -inf * ones;
    DM ub_y = inf * ones;
    DM lb_theta = -M_PI * ones;
    DM ub_theta = M_PI * ones;
    DM lb_v = 0.0 * ones;
    DM ub_v = calibration_.max_v * ones;
    DM lb_delta = -calibration_.max_delta * ones;
    DM ub_delta = calibration_.max_delta * ones;
    DM lb_a = calibration_.min_a * ones;
    DM ub_a = calibration_.max_a * ones;
    DM lb_omega = -calibration_.max_omega * ones;
    DM ub_omega = calibration_.max_omega * ones;
    DM lb_jerk = -calibration_.max_j * ones;
    DM ub_jerk = calibration_.max_j * ones;

    lb_x(0) = ub_x(0) = ref.points[0].x;
    lb_y(0) = ub_y(0) = ref.points[0].y;
    lb_theta(0) = ub_theta(0) = ref.points[0].h;
    lb_v(0) = ub_v(0) = ref.points[0].v;
    lb_a(0) = ub_a(0) = ref.points[0].a;

    for (size_t i = 0; i < cubes.size() && static_cast<int>(i) < n; ++i) {
      lb_x(i) = std::max(static_cast<double>(lb_x(i)), cubes[i].lb.x);
      ub_x(i) = std::min(static_cast<double>(ub_x(i)), cubes[i].ub.x);
      lb_y(i) = std::max(static_cast<double>(lb_y(i)), cubes[i].lb.y);
      ub_y(i) = std::min(static_cast<double>(ub_y(i)), cubes[i].ub.y);
    }

    DM guess_x(n, 1), guess_y(n, 1), guess_theta(n, 1), guess_v(n, 1), guess_a(n, 1);
    DM guess_delta = DM::zeros(n, 1), guess_omega = DM::zeros(n, 1), guess_jerk = DM::zeros(n, 1);

    const bool use_warm_start = calibration_.warm_start_enable &&
                                 !prev_optimal_.points.empty() &&
                                 static_cast<int>(prev_optimal_.points.size()) == n;
    if (use_warm_start) {
      const double dx = ref.points[0].x - prev_ego_px_;
      for (int i = 0; i < n; ++i) {
        guess_x(i) = prev_optimal_.points[i].x + dx;
        guess_y(i) = prev_optimal_.points[i].y;
        guess_theta(i) = prev_optimal_.points[i].h;
        guess_v(i) = prev_optimal_.points[i].v;
        guess_a(i) = prev_optimal_.points[i].a;
      }
    } else {
      for (int i = 0; i < n; ++i) {
        guess_x(i) = ref.points[i].x;
        guess_y(i) = ref.points[i].y;
        guess_theta(i) = ref.points[i].h;
        guess_v(i) = ref.points[i].v;
        guess_a(i) = ref.points[i].a;
      }
    }

    DMDict arg, res;
    arg["lbx"] = DM::vertcat({lb_x, lb_y, lb_theta, lb_v, lb_delta, lb_a, lb_omega, lb_jerk});
    arg["ubx"] = DM::vertcat({ub_x, ub_y, ub_theta, ub_v, ub_delta, ub_a, ub_omega, ub_jerk});
    arg["x0"] = DM::vertcat({guess_x, guess_y, guess_theta, guess_v, guess_delta, guess_a, guess_omega, guess_jerk});

    DM ref_x(n, 1), ref_y(n, 1), ref_theta(n, 1);
    DM cube_center_x(n, 1), cube_center_y(n, 1);
    for (int i = 0; i < n; ++i) {
      ref_x(i) = ref.points[i].x;
      ref_y(i) = ref.points[i].y;
      ref_theta(i) = ref.points[i].h;

      if (cubes.empty()) {
        cube_center_x(i) = ref.points[i].x;
        cube_center_y(i) = ref.points[i].y;
      } else {
        const int cube_idx = std::min<int>(static_cast<int>(cubes.size()) - 1, i);
        cube_center_x(i) = 0.5 * (cubes[cube_idx].lb.x + cubes[cube_idx].ub.x);
        cube_center_y(i) = 0.5 * (cubes[cube_idx].lb.y + cubes[cube_idx].ub.y);
      }
    }
    arg["p"] = DM::vertcat({calibration_.weight_penalty, ref_x, ref_y, ref_theta,
                             cube_center_x, cube_center_y});

    res = solver_(arg);
    DM opt = res.at("x");

    for (int i = 0; i < n; ++i) {
      out.points[i].x = static_cast<double>(opt(i));
      out.points[i].y = static_cast<double>(opt(n + i));
      out.points[i].h = static_cast<double>(opt(2 * n + i));
      out.points[i].v = static_cast<double>(opt(3 * n + i));
      out.points[i].a = static_cast<double>(opt(5 * n + i));
    }

    prev_optimal_ = out;
    prev_ego_px_ = ref.points[0].x;

    return out;
  } catch (...) {
  }
#endif

  // Fallback: Parametric trajectory optimization without CasADi
  const int stride = static_cast<int>(calibration_.sim_ts / calibration_.traj_ts);
    const double center_pull_x = utils::clamp(
      calibration_.weight_cube_center_x /
        (calibration_.weight_cube_center_x + calibration_.weight_x + 1e-9),
      0.0, 0.35);
    const double center_pull_y = utils::clamp(
      calibration_.weight_cube_center_y /
        (calibration_.weight_cube_center_y + calibration_.weight_y + 1e-9),
      0.0, 0.35);
  for (size_t i = 0; i < out.points.size(); ++i) {
    if (i == 0) {
      continue;
    }

    const int cube_idx = std::min<int>(cubes.size() - 1, static_cast<int>(i / std::max(1, stride)));
    if (cube_idx >= 0 && !cubes.empty()) {
      out.points[i].x = utils::clamp(out.points[i].x, cubes[cube_idx].lb.x, cubes[cube_idx].ub.x);
      out.points[i].y = utils::clamp(out.points[i].y, cubes[cube_idx].lb.y, cubes[cube_idx].ub.y);

      const double center_x = 0.5 * (cubes[cube_idx].lb.x + cubes[cube_idx].ub.x);
      const double center_y = 0.5 * (cubes[cube_idx].lb.y + cubes[cube_idx].ub.y);
      out.points[i].x += center_pull_x * (center_x - out.points[i].x);
      out.points[i].y += center_pull_y * (center_y - out.points[i].y);
    }

    out.points[i].x = 0.6 * out.points[i].x + 0.25 * out.points[i - 1].x +
                      (i > 1 ? 0.15 * out.points[i - 2].x : 0.0);
    out.points[i].y = 0.6 * out.points[i].y + 0.25 * out.points[i - 1].y +
                      (i > 1 ? 0.15 * out.points[i - 2].y : 0.0);

    const double dx = out.points[i].x - out.points[i - 1].x;
    const double dy = out.points[i].y - out.points[i - 1].y;
    if (std::hypot(dx, dy) > 1e-6) {
      out.points[i].h = utils::normalizeAngle(std::atan2(dy, dx));
    } else {
      out.points[i].h = out.points[i - 1].h;
    }

    out.points[i].v = utils::clamp(dx / calibration_.traj_ts, 0.0, calibration_.max_v);

    if (i > 0) {
      out.points[i].a = utils::clamp((out.points[i].v - out.points[i - 1].v) / calibration_.traj_ts,
                              calibration_.min_a, calibration_.max_a);
    }
  }

  for (size_t i = 1; i < out.points.size() - 1; ++i) {
    out.points[i].v = 0.5 * out.points[i].v + 0.25 * out.points[i - 1].v + 0.25 * out.points[i + 1].v;
  }

  for (size_t i = 1; i < out.points.size(); ++i) {
    out.points[i].a = utils::clamp((out.points[i].v - out.points[i - 1].v) / calibration_.traj_ts,
                            calibration_.min_a, calibration_.max_a);
  }

  return out;
}

Trajectory TrajectoryPlanner::plan(const Trajectory& coarse_ego,
                                   const std::vector<Trajectory>& coarse_agents,
                                   const std::vector<Vehicle>& vehicles,
                                   const std::vector<Lane>& lanes) const {
  if (coarse_ego.points.empty() || vehicles.empty() || lanes.empty()) {
    return coarse_ego;
  }

  auto ref = initGuess(coarse_ego, vehicles.front());

  const auto t_cube0 = std::chrono::steady_clock::now();
  auto cubes = generateDriveCubes(ref, coarse_agents, vehicles, lanes);
  last_cubes_ = cubes;
  const auto t_cube1 = std::chrono::steady_clock::now();
  const double cube_ms = std::chrono::duration<double, std::milli>(t_cube1 - t_cube0).count();
  RCLCPP_INFO(getLogger(), "DriveCube generation: %.2f ms (%zu cubes)", cube_ms, cubes.size());

  const auto t_opt0 = std::chrono::steady_clock::now();
  auto result = optimizeTrajectory(ref, cubes);
  const auto t_opt1 = std::chrono::steady_clock::now();
  const double opt_ms = std::chrono::duration<double, std::milli>(t_opt1 - t_opt0).count();
  RCLCPP_INFO(getLogger(), "Optimizer: %.2f ms (%s)",
           opt_ms,
#ifdef HAS_CASADI
           "CasADi/IPOPT"
#else
           "fallback-smoother"
#endif
           );
  return result;
}

}  // namespace lcs
