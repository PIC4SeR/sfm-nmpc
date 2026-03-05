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


#ifndef NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "sfm_nmpc/tools/motion_model.hpp"
#include "sfm_nmpc/tools/regulated_pure_pursuit.hpp"
#include "nav_msgs/msg/path.h"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_core
{
class GoalChecker;
}

namespace sfm_nmpc
{

/**
 * @class nav2_path_trajectorizer::PathTrajectorizer
 * @brief Regulated pure pursuit controller plugin
 */
class PathTrajectorizer
{
public:
  /**
   * @brief Constructor for nav2_path_trajectorizer::PathTrajectorizer
   */
  PathTrajectorizer();

  /**
   * @brief Destrructor for nav2_path_trajectorizer::PathTrajectorizer
   */
  ~PathTrajectorizer();

  /**
   * @brief Inject a motion model to integrate the robot state.
   * @param motion_model Concrete kinematic model; defaults to planar if nullptr is passed.
   */
  inline void setMotionModel(std::unique_ptr<MotionModel> motion_model)
  {
    if (motion_model)
    {
      motion_model_ = std::move(motion_model);
    }
    else
    {
      motion_model_ = std::make_unique<HolonomicMotionModel>(MotionModel::Limits{});
    }
  }

  /**
   * @brief Configure smoother parameters and member variables
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   */
  void configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf);

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup();

  /**
   * @brief Activate controller state machine
   */
  void activate();

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate();

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be optimized
   * @param path_robot_pose, pose of the robot in the same frame of the path
   * @return trajectorized path
   */
  bool trajectorize(nav_msgs::msg::Path& path, const geometry_msgs::msg::PoseStamped& path_robot_pose,
                    const geometry_msgs::msg::Twist& speed,
                    std::vector<geometry_msgs::msg::TwistStamped>& cmds,
                    nav2_core::GoalChecker* goal_checker = nullptr);

  float inline getTimeStep()
  {
    return time_step_;
  }

  inline void resetLastCommand()
  {
    last_cmd_ = geometry_msgs::msg::Twist();
    have_last_cmd_ = false;
  }

protected:
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const nav_msgs::msg::Path& path, double rx, double ry,
                                                    double rtheta) const;
  int findWaypointIndex(const nav_msgs::msg::Path& path, double rx, double ry, double target_dist) const;
  rclcpp::Time applyMotionModel(geometry_msgs::msg::PoseStamped& robot_pose, const geometry_msgs::msg::Twist& cmd,
                                const geometry_msgs::msg::Twist& prev_cmd, geometry_msgs::msg::Twist& applied_cmd
                                ) const;
  double computeTargetLinearVelocity(double curvature_measure) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_{ rclcpp::get_logger("PathTrajectorizer") };
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_;
  double waypoint_dist_tol_;
  double lookahead_dist_;
  bool allow_reverse_;
  double reverse_heading_threshold_;
  double max_reverse_speed_;
  double max_linear_vel_;
  double max_angular_vel_;
  double time_step_;
  double max_steps_;
  double max_linear_accel_;
  double max_angular_accel_;
  double min_approach_linear_velocity_{ 0.05 };
  bool use_interpolation_{ false };
  bool use_rotate_to_heading_{ true };
  double rotate_to_heading_angular_vel_{ 0.75 };
  double rotate_to_heading_min_angle_{ 1.0 };

  std::string base_frame_;
  geometry_msgs::msg::Twist last_cmd_;
  bool have_last_cmd_{ false };
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> received_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> computed_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> lookahead_marker_pub_;
  std::unique_ptr<MotionModel> motion_model_;
  RegulatedPurePursuit pure_pursuit_{};
};

}  // namespace sfm_nmpc

#endif  // NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
