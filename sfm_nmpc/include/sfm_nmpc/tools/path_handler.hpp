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


#ifndef MPC_SFM_MOTION_MODEL__PATH_HANDLER_HPP_
#define MPC_SFM_MOTION_MODEL__PATH_HANDLER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace mpc
{

/**
 * @class mpc::PathHandler
 * @brief Handles input paths to transform them to local frames required
 */
class PathHandler
{
public:
  /**
   * @brief Constructor for nav2_graceful_controller::PathHandler
   */
  PathHandler(tf2::Duration transform_tolerance, std::shared_ptr<tf2_ros::Buffer> tf,
              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Destructor for nav2_graceful_controller::PathHandler
   */
  ~PathHandler() = default;

  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for motionTarget
   * Points ineligible to be selected as a motion target point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @param max_robot_pose_search_dist Distance to search for matching nearest path point
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose,
                                          double max_robot_pose_search_dist);

  /**
   * @brief Sets the global plan
   *
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path& path);

  /**
   * @brief Resets the pruned plan back into the global plan.
   */
  void resetPlan();

  /**
   * @brief Gets the current global plan
   */
  nav_msgs::msg::Path getPlan()
  {
    return global_plan_;
  }

protected:
  rclcpp::Duration transform_tolerance_{ 0, 0 };
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path pruned_plan_;
  rclcpp::Logger logger_{ rclcpp::get_logger("PathHandler") };
};

}  // namespace mpc

#endif  // MPC_SFM_MOTION_MODEL__PATH_HANDLER_HPP_
