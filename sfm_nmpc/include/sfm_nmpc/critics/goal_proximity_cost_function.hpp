// Copyright 2026 PIC4SeR - Politecnico di Torino and SRL - Service Robotics Lab, Pablo de Olavide University
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#ifndef MPC_SFM_MOTION_MODEL__GOAL_PROXIMITY_COST_FUNCTION_HPP_
#define MPC_SFM_MOTION_MODEL__GOAL_PROXIMITY_COST_FUNCTION_HPP_

#include <algorithm>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"

#include "sfm_nmpc/update_state.hpp"

namespace sfm_nmpc
{

/**
 * @brief Logarithmic attractive potential toward the final global goal.
 *
 * Uses the potential  U(d) = log(1 + d / ε)  where d is the Euclidean
 * distance to the goal and ε = decay_distance controls the transition
 * from linear to logarithmic behaviour.
 *
 * Properties:
 *   • U(0) = 0  →  global minimum exactly at the goal.
 *   • ∇U = 1/(d + ε)  →  gradient always points toward the goal.
 *   • For d ≪ ε the potential is approximately linear  (d/ε),
 *     giving a strong, predictable pull near the goal.
 *   • For d ≫ ε it grows as log(d/ε), so it never overwhelms
 *     path-tracking or obstacle-avoidance costs at large range.
 *   • Smooth everywhere (no sigmoid, no barrier).
 *
 * The activation_radius parameter is kept in the constructor signature
 * for backward API compatibility but is not used.
 */
class GoalProximityCost
{
public:
  using GoalProximityCostFunction = ceres::DynamicAutoDiffCostFunction<GoalProximityCost>;

  GoalProximityCost(double weight, double activation_radius, double decay_distance,
                    const geometry_msgs::msg::Pose& goal_pose, const geometry_msgs::msg::Pose& robot_init,
                    unsigned int current_position, double time_step, unsigned int control_horizon,
                    unsigned int block_length)
    : weight_(weight),
      activation_radius_(activation_radius),
      decay_distance_(std::max(decay_distance, 1e-3)),
      goal_pose_(goal_pose),
      robot_init_(robot_init),
      current_position_(current_position),
      time_step_(time_step),
      control_horizon_(control_horizon),
      block_length_(block_length)
  {
  }

  inline static GoalProximityCostFunction* Create(double weight, double activation_radius, double decay_distance,
                                                  const geometry_msgs::msg::Pose& goal_pose,
                                                  const geometry_msgs::msg::Pose& robot_init,
                                                  unsigned int current_position, double time_step,
                                                  unsigned int control_horizon, unsigned int block_length)
  {
    return new GoalProximityCostFunction(new GoalProximityCost(weight, activation_radius, decay_distance, goal_pose,
                                                               robot_init, current_position, time_step,
                                                               control_horizon, block_length));
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    auto [new_position_x, new_position_y, new_position_orientation] = computeUpdatedStateRedux(
        robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);
    (void)new_position_orientation;

    T goal_x = (T)goal_pose_.position.x;
    T goal_y = (T)goal_pose_.position.y;
    T dx = goal_x - new_position_x;
    T dy = goal_y - new_position_y;

    // Smooth distance: avoids zero-norm gradient issues
    T dist = ceres::sqrt(dx * dx + dy * dy + T(1e-12));

    // Logarithmic attractive potential:  U(d) = log(1 + d / ε)
    //   • residual = 0 at the goal  (d = 0)
    //   • gradient ∝ 1/(d + ε), always pointing toward the goal
    //   • no sigmoid → no cost barrier
    T epsilon = (T)decay_distance_;
    residuals[0] = (T)weight_ * ceres::log(T(1.0) + dist / epsilon);
    return true;
  }

private:
  double weight_;
  double activation_radius_;  // kept for API compat, unused
  double decay_distance_;     // ε: linear-to-log transition scale
  geometry_msgs::msg::Pose goal_pose_;
  geometry_msgs::msg::Pose robot_init_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
};

}  // namespace sfm_nmpc

#endif  // MPC_SFM_MOTION_MODEL__GOAL_PROXIMITY_COST_FUNCTION_HPP_
