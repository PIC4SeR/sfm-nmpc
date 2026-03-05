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


#ifndef MPC_SFM_MOTION_MODEL__OPTIMIZER_HPP_
#define MPC_SFM_MOTION_MODEL__OPTIMIZER_HPP_

#include <math.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "ceres/cubic_interpolation.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// cost functions
#include "sfm_nmpc/critics/agent_angle_cost_function.hpp"
#include "sfm_nmpc/critics/crossing_cost_function.hpp"
#include "sfm_nmpc/critics/angle_cost_function.hpp"
#include "sfm_nmpc/critics/curvature_cost_function.hpp"
#include "sfm_nmpc/critics/distance_cost_function.hpp"
#include "sfm_nmpc/critics/goal_align_cost_function.hpp"
#include "sfm_nmpc/critics/goal_proximity_cost_function.hpp"
#include "sfm_nmpc/critics/obstacle_cost_function.hpp"
#include "sfm_nmpc/critics/social_work_cost_function.hpp"
#include "sfm_nmpc/critics/velocity_cost_function.hpp"
#include "sfm_nmpc/critics/velocity_feasibility_cost_function.hpp"
#include "sfm_nmpc/critics/proxemics_cost_function.hpp"

#include "sfm_nmpc/sfm.hpp"
#include "sfm_nmpc/trajectory_memory.hpp"
#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "people_msgs/msg/people.hpp"
#include "sfm_nmpc/tools/type_definitions.hpp"

namespace sfm_nmpc
{

struct OptimizerParams
{
  OptimizerParams()
  {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode* node, const std::string& name);
  const std::map<std::string, ceres::LinearSolverType> solver_types = {
    { "DENSE_SCHUR", ceres::DENSE_SCHUR },
    { "SPARSE_SCHUR", ceres::SPARSE_SCHUR },
    { "DENSE_NORMAL_CHOLESKY", ceres::DENSE_NORMAL_CHOLESKY },
    { "DENSE_QR", ceres::DENSE_QR },
    { "SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY }
  };

  std::string linear_solver_type;

  double param_tol;     // Ceres default: 1e-8
  double fn_tol;        // Ceres default: 1e-6
  double gradient_tol;  // Ceres default: 1e-10
  double socialwork_w_;
  double distance_w_;
  double velocity_w_;
  double angle_w_;
  double agent_angle_w_;
  double velocity_alignment_w_;
  double crossing_w_;
  double crossing_bearing_w_;
  double velocity_feasibility_w_;
  double goal_align_w_;
  double obstacle_w_;
  double proxemics_w_;
  double goal_proximity_w_;
  double goal_proximity_activation_radius_;
  double goal_proximity_decay_distance_;
  bool use_social_work_cost;
  bool use_social_angle_cost;
  bool use_social_crossing_cost;
  bool use_social_proxemics_cost;
  bool use_social_path_follow_cost;
  bool use_social_path_align_cost;
  int max_agents;
  float current_path_w;
  float current_cmds_w;
  float max_time;
  int discretization_;
  int control_horizon_;
  int parameter_block_length_;
  bool debug;
  int max_iterations;
  double max_linear_vel;
  double min_linear_vel;
  double max_angular_vel;
  double min_angular_vel;
  double desired_linear_vel;
  double agent_velocity_bound;
  double stationary_agent_velocity_bound;
  double stationary_agent_speed_threshold;
};

/**
 * @brief Optimizer class for the social MPC controller
 */
class Optimizer
{
public:
  // x, y
  struct position
  {
    double params[2];
  };

  // x, y, lv, av
  struct posandvel
  {
    double params[4];
  };

  // lv, av
  struct vel
  {
    double params[2];
  };

  // t, yaw
  struct heading
  {
    double params[2];
  };
  struct linear_velocity
  {
    double params[1];
  };
  struct angular_velocity
  {
    double params[1];
  };
  Optimizer();

  /**
   * @brief Destrructor for
   * sfm_nmpc::MPCSFMMotionModel
   */
  ~Optimizer();

  /**
   * @brief Initialization of the optimizer
   * @param params OptimizerParam struct
   */
  void initialize(const OptimizerParams params);

  /**
   * @brief Optimize the path using the social MPC controller
   * @param path The path to optimize
   * @param people_proj Projected people positions
   * @param costmap Costmap for obstacle avoidance
   * @param obstacles Obstacle distances
   * @param cmds Commands to execute
   * @param people People detected in the environment
   * @param speed Current robot speed
   * @param time_step Time step for discretization
   * @return true if optimization succeeded
   * @return false if optimization failed
   */
  bool optimize(nav_msgs::msg::Path& path, AgentsTrajectories& people_proj, const nav2_costmap_2d::Costmap2D* costmap,
                std::vector<geometry_msgs::msg::TwistStamped>& cmds, const people_msgs::msg::People& people,
                const geometry_msgs::msg::Twist& speed, const float time_step,
                const geometry_msgs::msg::PoseStamped& goal_pose);

private:
  /**
   * @brief Convert people messages to agent status
   * @param people People messages
   * @return Vector of agent statuses
   */
  AgentsStates people_to_status(const people_msgs::msg::People& people);

  /**
   * @brief Format path and commands for optimization
   * @param path Current path
   * @param previous_path Previous path
   * @param cmds Current commands
   * @param previous_cmds Previous commands
   * @param speed Current robot speed
   * @param current_path_w Weight for current path
   * @param current_cmds_w Weight for current commands
   * @param maxtime Maximum time horizon
   * @param timestep Time step
   * @return Vector of agent statuses
   */
  AgentTrajectory format_to_optimize(nav_msgs::msg::Path& path, const nav_msgs::msg::Path& previous_path,
                                     const std::vector<geometry_msgs::msg::TwistStamped>& cmds,
                                     const std::vector<geometry_msgs::msg::TwistStamped>& previous_cmds,
                                     const geometry_msgs::msg::Twist& speed, const float current_path_w,
                                     const float current_cmds_w, const float maxtime, const float timestep);


  /**
   * @brief Compute obstacle position relative to agent
   * @param apos Agent position
   * @param od Obstacle distances
   * @return Vector to obstacle
   */
  Eigen::Vector2d computeObstacle(const Eigen::Vector2d& apos, const obstacle_distance_msgs::msg::ObstacleDistance& od);

  bool debug_;
  unsigned int control_horizon_;
  unsigned int parameter_block_length_;
  float max_time;
  double obstacle_w_;
  double velocity_feasibility_w_;
  double agent_angle_w_;
  double velocity_alignment_w_;
  double crossing_w_;
  double crossing_bearing_w_;
  double angle_w_;
  double distance_w_;
  double socialwork_w_;
  double goal_align_w_;
  double velocity_w_;
  double curvature_w_;
  double proxemics_w_;
  double curvature_angle_min_;
  double goal_proximity_w_;
  double goal_proximity_activation_radius_;
  double goal_proximity_decay_distance_;
  bool use_social_work_cost_{true};
  bool use_social_angle_cost_{true};
  bool use_social_crossing_cost_{true};
  bool use_social_proxemics_cost_{true};
  bool use_social_path_follow_cost_{true};
  bool use_social_path_align_cost_{true};
  float current_path_w;
  float current_cmds_w;
  double max_linear_vel_;
  double min_linear_vel_;
  double max_angular_vel_;
  double min_angular_vel_;
  double desired_linear_vel_;
  double agent_velocity_bound_;
  double stationary_agent_velocity_bound_;
  double stationary_agent_speed_threshold_;
  int max_agents_;
  ceres::Solver::Options options_;
  std::shared_ptr<ceres::Grid2D<u_char>> costmap_grid_;
  std::shared_ptr<ceres::Grid2D<float>> obs_grid_;
  std::string frame_;
  rclcpp::Time path_time_;
};

}  // namespace sfm_nmpc

#endif  // MPC_SFM_MOTION_MODEL__OPTIMIZER_HPP_
