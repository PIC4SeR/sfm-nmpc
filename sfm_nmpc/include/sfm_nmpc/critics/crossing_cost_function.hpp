#ifndef MPC_SFM_MOTION_MODEL__CROSSING_COST_FUNCTION_HPP_
#define MPC_SFM_MOTION_MODEL__CROSSING_COST_FUNCTION_HPP_

#include <sfm_nmpc/update_state.hpp>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
#include "sfm_nmpc/tools/type_definitions.hpp"

/**
 * @file crossing_cost_function.hpp
 * @brief Crossing cost that forces the robot to YIELD and STEER behind
 *        a crossing agent.
 *
 * Two components, both gated by sin²(θ_robot − θ_agent) which peaks at
 * 90° crossings and vanishes for same / opposite directions:
 *
 *   1. **Speed penalty** `v_robot · sin²(Δθ)`:
 *      DIRECT gradient on the linear velocity parameter → deceleration.
 *
 *   2. **Steering penalty** `softplus(cross · ω · scale)`:
 *      Uses the 2-D cross product of (agent−robot) × agent_heading to
 *      determine which side the agent approaches from, then penalises
 *      omega in the WRONG direction.  Provides a DIRECT gradient on the
 *      angular velocity parameter → 150× stronger than indirect position
 *      chain.
 *
 *      cross < 0 ⟹ agent from right ⟹ turn LEFT  (ω > 0)
 *      cross > 0 ⟹ agent from left  ⟹ turn RIGHT (ω < 0)
 *      softplus(cross · ω) is large when cross and ω have the same sign
 *      (wrong direction) and near zero when they have opposite signs
 *      (correct direction).
 *
 * Both are scaled by exponential distance decay (safe distance 3 m).
 *
 *   residual = weight · dist_decay ·
 *              ( v · sin²(Δθ)  +  steer_w · softplus(cross·ω·s) · sin²(Δθ) )
 */

namespace sfm_nmpc
{

class CrossingCost
{
public:
  using CrossingCostFunction = ceres::DynamicAutoDiffCostFunction<CrossingCost>;

  CrossingCost(double weight, double bearing_weight,
               const AgentsStates& agents_init,
               const geometry_msgs::msg::Pose& robot_init,
               unsigned int current_position, double time_step,
               unsigned int control_horizon, unsigned int block_length);

  /**
   * @brief Factory method for Ceres.
   */
  inline static CrossingCostFunction* Create(double weight, double bearing_weight,
                                              const AgentsStates& agents_init,
                                              const geometry_msgs::msg::Pose& robot_init,
                                              unsigned int current_position, double time_step,
                                              unsigned int control_horizon, unsigned int block_length)
  {
    return new CrossingCostFunction(new CrossingCost(weight, bearing_weight,
                                                     agents_init, robot_init,
                                                     current_position, time_step,
                                                     control_horizon, block_length));
  }

  /**
   * @brief Ceres autodiff functor.
   *
   * 1. Forward-simulates robot + agents via SFM.
   * 2. Finds the closest valid moving agent within safe distance.
   * 3. Computes crossing_intensity = sin²(θ_robot − θ_agent).
   * 4. Speed component: v × sin²(Δθ) → direct gradient on v.
   * 5. Steering component: softplus(cross · ω · scale) × sin²(Δθ)
   *    → direct gradient on ω pushing robot BEHIND the agent.
   * 6. Scales by exponential distance decay.
   */
  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    Eigen::Matrix<T, 6, 3> agents_ = original_agents_.template cast<T>();
    auto [new_position_x, new_position_y, new_position_orientation, agents] =
        computeSFMState(robot_init_, agents_, parameters, time_step_, current_position_, control_horizon_,
                        block_length_);

    // ---- Current parameter block: [v, ω] ----
    unsigned int block_index;
    if (current_position_ < control_horizon_)
      block_index = current_position_ / block_length_;
    else
      block_index = (control_horizon_ - 1) / block_length_;
    T robot_v = parameters[block_index][0];
    T robot_omega = parameters[block_index][1];

    // ---- Find closest valid, moving agent ----
    int closest_index = -1;
    T closest_distance_squared = T(9999.0);
    for (unsigned int i = 0; i < agents.cols(); i++)
    {
      if (agents(3, i) == (T)-1.0)
        continue;
      T dx = agents(0, i) - new_position_x;
      T dy = agents(1, i) - new_position_y;
      T distance_squared = dx * dx + dy * dy;
      if (distance_squared < closest_distance_squared && agents(4, i) > T(0.05))
      {
        closest_distance_squared = distance_squared;
        closest_index = i;
      }
    }

    if (closest_index < 0 || closest_distance_squared > safe_distance_squared_)
    {
      residuals[0] = T(0.0);
      return true;
    }

    // ---- Crossing geometry: sin²(Δθ) peaks at 90° crossings ----
    T agent_heading = agents(2, closest_index);
    T heading_diff = new_position_orientation - agent_heading;
    T sin_diff = ceres::sin(heading_diff);
    T crossing_intensity = sin_diff * sin_diff;  // 1.0 at 90°, 0.0 at 0°/180°

    // ---- Component 1: Speed penalty (direct gradient on v) ----
    T speed_component = robot_v * crossing_intensity;

    // ---- Component 2: Steering penalty (direct gradient on ω) ----
    // 2D cross product: (agent − robot) × agent_heading_dir
    //   cross < 0 → agent approaches from right → robot should turn LEFT  (ω > 0)
    //   cross > 0 → agent approaches from left  → robot should turn RIGHT (ω < 0)
    // softplus(cross · ω) penalises ω in the WRONG direction.
    T agent_heading_dx = ceres::cos(agent_heading);
    T agent_heading_dy = ceres::sin(agent_heading);
    T to_agent_x = agents(0, closest_index) - new_position_x;
    T to_agent_y = agents(1, closest_index) - new_position_y;
    T cross = to_agent_x * agent_heading_dy - to_agent_y * agent_heading_dx;

    T steer_scale = T(3.0);
    T k = T(5.0);
    T steer_penalty = ceres::log(T(1.0) + ceres::exp(k * cross * robot_omega * steer_scale)) / k;
    T steer_component = steer_penalty * crossing_intensity;

    // ---- Exponential distance decay ----
    T dist_decay = ceres::exp(-closest_distance_squared / T(safe_distance_squared_));

    residuals[0] = (T)weight_ * dist_decay *
                   (speed_component + (T)bearing_weight_ * steer_component);
    return true;
  }

private:
  double weight_;
  double bearing_weight_;
  AgentsStates agents_init_;
  geometry_msgs::msg::Pose robot_init_;
  Eigen::Matrix<double, 6, 3> original_agents_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  double safe_distance_squared_;
};

}  // namespace sfm_nmpc

#endif  // MPC_SFM_MOTION_MODEL__CROSSING_COST_FUNCTION_HPP_
