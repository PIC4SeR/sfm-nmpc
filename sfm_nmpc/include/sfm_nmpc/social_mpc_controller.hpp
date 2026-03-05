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


#ifndef MPC_SFM_MOTION_MODEL__MPC_CONTROLLER_HPP_
#define MPC_SFM_MOTION_MODEL__MPC_CONTROLLER_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"
#include "sfm_nmpc/obstacle_distance_interface.hpp"
#include "sfm_nmpc/optimizer.hpp"
#include "sfm_nmpc/path_trajectorizer.hpp"
#include "sfm_nmpc/people_interface.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "people_msgs/msg/people.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "sfm_nmpc/tools/path_handler.hpp"
#include "sfm_nmpc/tools/type_definitions.hpp"

namespace sfm_nmpc
{

/**
 * @class sfm_nmpc::MPCSFMMotionModel
 * @brief social mpc controller plugin
 */
class MPCSFMMotionModel : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for
   * sfm_nmpc::MPCSFMMotionModel
   */
  MPCSFMMotionModel() = default;

  /**
   * @brief Destrructor for
   * sfm_nmpc::MPCSFMMotionModel
   */
  ~MPCSFMMotionModel() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of node
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with
   * possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param results   Output param, if not NULL, will be filled in with full
   * evaluation results
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                           const geometry_msgs::msg::Twist& velocity,
                                                           nav2_core::GoalChecker* goal_checker) override;
  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path& path) override;

  void publish_people_traj(const AgentsTrajectories& people, const std_msgs::msg::Header& header);

protected:
  // /**
  //  * @brief Transforms global plan into same frame as pose, clips far away poses
  //  * and possibly prunes passed poses
  //  * @param pose pose to transform
  //  * @return Path in new frame
  //  */
  // nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped& in_pose,
                     geometry_msgs::msg::PoseStamped& out_pose) const;

  /**
   * @brief
   *
   * @param frame
   * @param in_point
   * @param out_point
   * @return true
   * @return false
   */
  bool transformPoint(const std::string frame, const geometry_msgs::msg::PointStamped& in_point,
                      geometry_msgs::msg::PointStamped& out_point) const;



  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::unique_ptr<PathTrajectorizer> trajectorizer_;
  std::unique_ptr<Optimizer> optimizer_;
  std::unique_ptr<PeopleInterface> people_interface_;
  std::unique_ptr<ObstacleDistInterface> obsdist_interface_;
  OptimizerParams optimizer_params_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_;
  rclcpp::Logger logger_{ rclcpp::get_logger("MPCSFMMotionModel") };

  double speed_limit;
  bool percentage;
  tf2::Duration transform_tolerance_;
  double max_robot_pose_search_dist_;
  double max_linear_vel_;
  double min_linear_vel_;
  double max_angular_vel_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_path_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> people_traj_pub_;
  std::unique_ptr<mpc::PathHandler> path_handler_;
};

}  // namespace sfm_nmpc

#endif  // MPC_SFM_MOTION_MODEL__MPC_CONTROLLER_HPP_
