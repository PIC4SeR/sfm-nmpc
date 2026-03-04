// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University and PIC4SeR - Politecnico di Torino
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MPC_SFM_MOTION_MODEL__AGENT_ANGLE_COST_FUNCTION_HPP_
#define MPC_SFM_MOTION_MODEL__AGENT_ANGLE_COST_FUNCTION_HPP_

#include <sfm_nmpc/update_state.hpp>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
#include "sfm_nmpc/tools/type_definitions.hpp"

/**
 * @file agent_angle_cost_function.hpp
 * @brief Defines the cost function for evaluating robot steering adjustments based on nearby agent orientations.
 *
 * This file implements the AgentAngleCost struct which serves as a functor for calculating a cost
 * that incentives certain angular deviations in a model predictive control (MPC) framework for social navigation.
 *
 * The cost is evaluated only when a nearby agent (satisfying certain activity and proximity thresholds)
 * is present. The function wraps angular differences to the range [-pi, pi] and adjusts its computation
 * based on the relative heading of the agent with respect to the robot's initial pose.
 *
 * The cost function leverages Ceres Solver for automatic differentiation and angle operations, and is
 * designed to be integrated within an optimization problem formulated to improve robot trajectory planning
 * in social environments.
 *
 * @note This implementation assumes that the state update computed by computeUpdatedStateRedux and other
 * necessary external functions or variables (e.g., tf2::getYaw) are defined elsewhere.
 *
 * @license Apache License, Version 2.0
 */

namespace sfm_nmpc
{

class AgentAngleCost
{
  /**
   * @class AgentAngleCost
   * @brief Functor for computing angular cost based on relative positions and orientations of agents.
   *
   * The AgentAngleCost computes the residual cost that penalizes the deviation of the robot's new orientation
   * from an expected angular value determined by the robot's initial orientation and the heading of a close-by agent.
   *
   * The function finds the nearest agent that satisfies a minimum linear velocity threshold, and if the distance is
   * within a predefined safe threshold, the cost is computed. The angular correction is applied differently based on
   * the relative orientation of the agent with respect to the robot’s initial yaw.
   *
   * @param weight A scaling factor for the cost.
   * @param agents_init A vector of agent statuses representing their initial state. Each agent status is a 6x1 Eigen
   * vector.
   * @param agents_zero A vector of baseline (zero or near-zero) agent statuses used for comparison.
   * @param robot_init The initial pose of the robot, which includes its position and orientation.
   * @param current_position The current index or time step in the planning horizon.
   * @param time_step The time interval between consecutive state updates.
   * @param control_horizon The total number of time steps considered in the MPC.
   * @param block_length The segmentation length used in state update computations.
   *
   * Member Variables:
   * - block_length_: Number of time steps in a block for updating the state.
   * - control_horizon_: Total number of steps in the MPC planning horizon.
   * - time_step_: Duration between state updates.
   * - current_position_: Index marking the current position in the planning sequence.
   * - safe_distance_: The threshold distance below which agent interactions trigger cost calculations.
   * - weight_: The overall scaling weight applied to the computed cost.
   * - robot_init_: The initial pose of the robot.
   * - agents_init_: List of initial statuses (poses, velocities, etc.) of the agents.
   */
public:
  using AgentAngleCostFunction = ceres::DynamicAutoDiffCostFunction<AgentAngleCost>;
  AgentAngleCost(double weight, double velocity_alignment_weight, const AgentsStates& agents_init,
                 const geometry_msgs::msg::Pose& robot_init, unsigned int current_position, double time_step,
                 unsigned int control_horizon, unsigned int block_length);

  /**
    * @brief Creates a Ceres cost function for the AgentAngleCost.
    *
    * This function is a factory method that constructs an instance of the
    * AgentAngleCostFunction using the provided parameters.

    * @param weight The weight for the cost function.
    * @param velocity_alignment_weight Weight for penalizing heading in the same direction as the agent's travel.
    * @param agents_init A vector of initial agent statuses.
    * @param robot_init The initial pose of the robot.
    * @param current_position The current position in the planning sequence.
    * @param time_step The time step for the MPC.
    * @param control_horizon The total number of time steps in the MPC.
    * @param block_length The length of the parameter block for the MPC.
    * @return A pointer to the created AgentAngleCostFunction instance.
    */

  inline static AgentAngleCostFunction* Create(double weight, double velocity_alignment_weight,
                                               const AgentsStates& agents_init,
                                               const geometry_msgs::msg::Pose& robot_init,
                                               unsigned int current_position, double time_step,
                                               unsigned int control_horizon, unsigned int block_length)
  {
    return new AgentAngleCostFunction(new AgentAngleCost(weight, velocity_alignment_weight, agents_init, robot_init,
                                                         current_position, time_step, control_horizon, block_length));
  }

  /**
   * @brief Operator to compute the cost based on the robot's state and agent positions.
   * @param T The type used for automatic differentiation (typically a double or Jet type from Ceres).
   * @param parameters Double pointer to robot state parameters.
   * @param residuals Pointer to the computed residual used in the optimization problem.
   *
   * The operator() function computes the updated state based on the input parameter block and evaluates
   * how much the robot's projected heading points toward a nearby agent. The cost has two components:
   *   1. Position alignment: penalizes heading toward the agent (softplus of cos(relative_angle))
   *   2. Velocity alignment: penalizes heading in the same direction as the agent's travel
   *   - Both use exponential distance decay, strongest when close
   *   - No hard left/right branching, avoiding flickering at boundaries
   * The gradient naturally steers the robot away from and opposite to the agent's travel direction.
   */
  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    Eigen::Matrix<T, 6, 3> agents_ = original_agents_.template cast<T>();
    auto [new_position_x, new_position_y, new_position_orientation, agents] =
        computeSFMState(robot_init_, agents_, parameters, time_step_, current_position_, control_horizon_,
                        block_length_);

    // Find closest valid, moving agent — measured from the PROJECTED robot position
    int closest_index = -1;
    T closest_distance_squared = T(9999.0);
    for (unsigned int i = 0; i < agents.cols(); i++)
    {
      if (agents(3, i) == (T)-1.0)  // Skip invalid agents
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

    // Angle from projected robot position to the closest agent
    T angle_to_agent = ceres::atan2(agents(1, closest_index) - new_position_y,
                                    agents(0, closest_index) - new_position_x);

    // Relative angle: how much the agent is in front of the robot
    // 0 = dead ahead, ±π = behind
    T rel_angle = ceres::atan2(ceres::sin(angle_to_agent - new_position_orientation),
                               ceres::cos(angle_to_agent - new_position_orientation));

    // Heading alignment: cos(rel_angle) = 1 when pointing at agent, -1 when away
    T alignment = ceres::cos(rel_angle);

    // Smooth ReLU (softplus): only penalize when robot heading points toward agent
    // softplus(x) = log(1 + exp(k*x)) / k  ≈  max(0, x) for large k
    // k=5 gives a smooth transition: at alignment=0 → ~0.14 (mild), at 1 → ~1.0 (full)
    T k = T(5.0);
    T active = ceres::log(T(1.0) + ceres::exp(k * alignment)) / k;

    // Exponential distance decay: strong when close, fades at safe distance
    T dist_decay = ceres::exp(-closest_distance_squared / T(safe_distance_squared_));

    // --- Component 2: penalize heading in the same direction as the agent's travel ---
    // Agent's heading from SFM state (row 2 = yaw)
    T agent_heading = agents(2, closest_index);

    // How aligned is the robot's heading with the agent's travel direction
    // +1 = same direction, -1 = opposite direction
    T velocity_alignment = ceres::cos(new_position_orientation - agent_heading);

    // Softplus: only penalize when heading in the same direction as the agent
    T velocity_active = ceres::log(T(1.0) + ceres::exp(k * velocity_alignment)) / k;

    residuals[0] = (T)weight_ * dist_decay * (active + (T)velocity_alignment_weight_ * velocity_active);
    return true;
  }

private:
  double weight_;
  double velocity_alignment_weight_;
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

#endif