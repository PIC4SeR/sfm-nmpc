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


#include "sfm_nmpc/social_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "angles/angles.h"
#include "tf2/utils.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT


namespace sfm_nmpc
{

void MPCSFMMotionModel::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                                    std::shared_ptr<tf2_ros::Buffer> tf,
                                    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  double transform_tolerance;
  
  // Declare the parameters
  declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_robot_pose_search_dist",
                                    rclcpp::ParameterValue(4.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_linear_vel", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.4));
  
  // Get the parameters
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  // Create the trajectorizer
  trajectorizer_ = std::make_unique<PathTrajectorizer>();
  trajectorizer_->configure(node, name, tf_);
  optimizer_ = std::make_unique<Optimizer>();
  optimizer_params_.get(node.get(), name);
  optimizer_->initialize(optimizer_params_);

  // people interface
  people_interface_ = std::make_unique<PeopleInterface>(parent);

  // path handler
  path_handler_ = std::make_unique<mpc::PathHandler>(transform_tolerance_, tf_, costmap_ros_);

  // obstacle distance transform
  obsdist_interface_ =
      std::make_unique<ObstacleDistInterface>(node, costmap_ros_->getGlobalFrameID(), tf_, transform_tolerance_);

  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

  people_traj_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("people_projected_trajectory", 1);
}

void MPCSFMMotionModel::cleanup()
{
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type"
              "sfm_nmpc::MPCSFMMotionModel",
              plugin_name_.c_str());
  local_path_pub_.reset();
  people_traj_pub_.reset();
}

void MPCSFMMotionModel::activate()
{
  RCLCPP_INFO(logger_,
              "Activating controller: %s of type "
              "sfm_nmpc::MPCSFMMotionModel",
              plugin_name_.c_str());
  trajectorizer_->activate();
  local_path_pub_->on_activate();
  people_traj_pub_->on_activate();
}

void MPCSFMMotionModel::deactivate()
{
  RCLCPP_INFO(logger_,
              "Deactivating controller: %s of type "
              "sfm_nmpc::MPCSFMMotionModel",
              plugin_name_.c_str());
  trajectorizer_->deactivate();
  local_path_pub_->on_deactivate();
  people_traj_pub_->on_deactivate();
}

void MPCSFMMotionModel::publish_people_traj(const AgentsTrajectories& people, const std_msgs::msg::Header& header)
{
  if (people.empty())
  {
    return;
  }
  // Ensure at least one agent list exists before indexing
  if (people[0].empty())
  {
    return;
  }
  // Create one marker for each person
  size_t npeople = people[0].size();
  visualization_msgs::msg::MarkerArray ma;
  for (size_t idx = 0; idx < npeople; idx++)
  {
    if (people[0][idx][3] != -1.0)
    {
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.type = m.LINE_STRIP;
      m.id = idx;
      m.action = m.ADD;
      m.scale.x = 0.05;
      m.color.a = 1.0;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      ma.markers.push_back(m);
    }
  }

  for (unsigned int stepi = 0; stepi < people.size(); stepi++)
  {
    int mi = 0;
    if (people[stepi].empty())
    {
      continue;
    }
    for (unsigned int personi = 0; personi < people[stepi].size(); personi++)
    {
      if (people[stepi][personi][3] != -1.0)
      {
        if (mi >= static_cast<int>(ma.markers.size()))
        {
          // No pre-created marker for this index; skip to avoid out-of-range access
          continue;
        }
        geometry_msgs::msg::Point point;
        point.x = people[stepi][personi][0];
        point.y = people[stepi][personi][1];
        point.z = 0.1;
        ma.markers[mi].points.push_back(point);
        mi++;
      }
    }
  }
  people_traj_pub_->publish(ma);
}

geometry_msgs::msg::TwistStamped MPCSFMMotionModel::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& robot_pose, const geometry_msgs::msg::Twist& speed,
    nav2_core::GoalChecker* goal_checker)
{
  // Use goal_checker to avoid unused parameter warning
  if (goal_checker == nullptr)
  {
    RCLCPP_WARN(logger_, "Goal checker is null");
  }
  nav_msgs::msg::Path transformed_plan =
      path_handler_->transformGlobalPlan(robot_pose, max_robot_pose_search_dist_);

  // Trajectorize the path
  if (transformed_plan.poses.empty())
  {
    RCLCPP_ERROR(logger_, "Transformed plan is empty, cannot compute velocity commands");
    geometry_msgs::msg::TwistStamped zero_vel;
    zero_vel.header = robot_pose.header;
    return zero_vel;
  }

  geometry_msgs::msg::PoseStamped global_goal_pose = transformed_plan.poses.back();
  nav_msgs::msg::Path traj_path = transformed_plan;
  std::vector<geometry_msgs::msg::TwistStamped> cmds;

  if (!trajectorizer_->trajectorize(traj_path, robot_pose, speed, cmds, goal_checker))
  {
    RCLCPP_ERROR(logger_, "Trajectorization of the path failed returning zero velocity");
    geometry_msgs::msg::TwistStamped zero_vel;
    zero_vel.header = robot_pose.header;
    return zero_vel;
  }
  std::vector<geometry_msgs::msg::TwistStamped> init_cmds = cmds;

  // Be careful, path and people must be in the same frame

  people_msgs::msg::People people = people_interface_->getPeople();
  if (people.people.empty())
  {
    RCLCPP_DEBUG(logger_, "People topic has no detections, skipping social critics");
  }

  if (people.header.frame_id != transformed_plan.header.frame_id)
  {
    // transform people to the global frame
    for (auto& p : people.people)
    {
      geometry_msgs::msg::PointStamped out_point;
      geometry_msgs::msg::PointStamped in_point;
      in_point.point = p.position;
      in_point.header = people.header;
      if (!transformPoint(transformed_plan.header.frame_id, in_point, out_point))
      {
        throw nav2_core::PlannerException("Unable to transform people point into plan's frame");
      }
      p.position = out_point.point;
    }
    people.header.frame_id = transformed_plan.header.frame_id;
  }

  float ts = trajectorizer_->getTimeStep();
  AgentsTrajectories projected_people;
  bool optimized = optimizer_->optimize(traj_path, projected_people, costmap_,cmds, people, speed, ts, global_goal_pose);
  if (!optimized)
  {
    RCLCPP_WARN(logger_, "Optimization failed, using initial commands");
    cmds = init_cmds;
  }
  publish_people_traj(projected_people, transformed_plan.header);
  local_path_pub_->publish(traj_path);

  if (cmds.empty())
  {
    RCLCPP_WARN(logger_, "Trajectorizer provided no commands, sending zero velocity");
    geometry_msgs::msg::TwistStamped zero_vel;
    zero_vel.header = robot_pose.header;
    return zero_vel;
  }

  // populate and return twist message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = cmds[0].header;
  double linear_cmd = std::clamp(cmds[0].twist.linear.x, -max_linear_vel_, max_linear_vel_);
  if (std::abs(linear_cmd) > 1e-6 && std::abs(linear_cmd) < min_linear_vel_)
  {
    linear_cmd = std::copysign(min_linear_vel_, linear_cmd);
  }
  double angular_cmd = std::clamp(cmds[0].twist.angular.z, -max_angular_vel_, max_angular_vel_);
  cmd_vel.twist.linear.x = linear_cmd;
  cmd_vel.twist.angular.z = angular_cmd;
  RCLCPP_DEBUG(logger_, "cmd_vel: %f, %f", cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
  return cmd_vel;
}

void MPCSFMMotionModel::setPlan(const nav_msgs::msg::Path& path)
{
  path_handler_->setPlan(path);
}

void MPCSFMMotionModel::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  double speed_limit_ = speed_limit;
  bool percentage_ = percentage;
  double throwaway_vel = 1;
  if (percentage_)
  {
    throwaway_vel *= (speed_limit_ / 100.0);
    RCLCPP_DEBUG(logger_, "Speed limit set as percentage: %f%%, resulting speed: %f", speed_limit_,
                  throwaway_vel);
  }
  else
  {
    throwaway_vel = speed_limit_;
    RCLCPP_DEBUG(logger_, "Speed limit set as absolute value: %f", throwaway_vel);
  }
}

bool MPCSFMMotionModel::transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped& in_pose,
                                        geometry_msgs::msg::PoseStamped& out_pose) const
{
  if (in_pose.header.frame_id == frame)
  {
    out_pose = in_pose;
    return true;
  }

  try
  {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

bool MPCSFMMotionModel::transformPoint(const std::string frame, const geometry_msgs::msg::PointStamped& in_point,
                                         geometry_msgs::msg::PointStamped& out_point) const
{
  try
  {
    tf_->transform(in_point, out_point, frame, transform_tolerance_);
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger_, "Exception in transformPoint: %s", ex.what());
  }
  return false;
}

}  // namespace sfm_nmpc

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(sfm_nmpc::MPCSFMMotionModel, nav2_core::Controller)
