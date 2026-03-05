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


#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "sfm_nmpc/tools/path_handler.hpp"

#include "nav2_core/exceptions.hpp"

namespace mpc
{

using nav2_util::geometry_utils::euclidean_distance;

PathHandler::PathHandler(tf2::Duration transform_tolerance, std::shared_ptr<tf2_ros::Buffer> tf,
                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  : transform_tolerance_(transform_tolerance), tf_buffer_(tf), costmap_ros_(costmap_ros)
{
}

nav_msgs::msg::Path PathHandler::transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose,
                                                     double max_robot_pose_search_dist)
{
  auto& global_plan_poses = global_plan_.poses;
  auto& pruned_plan_poses = pruned_plan_.poses;

  // Check first if the plan is empty
  if (global_plan_poses.empty())
  {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // Let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav_2d_utils::transformPose(tf_buffer_, global_plan_.header.frame_id, pose, robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // Find the first pose in the global plan that's further than max_robot_pose_search_dist
  // from the robot using integrated distance
  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
      global_plan_poses.begin(), global_plan_poses.end(), max_robot_pose_search_dist);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin = nav2_util::geometry_utils::min_by(
      global_plan_poses.begin(), closest_pose_upper_bound,
      [&robot_pose](const geometry_msgs::msg::PoseStamped& ps) { return euclidean_distance(robot_pose, ps); });
  const geometry_msgs::msg::PoseStamped transformation_begin_pose = *transformation_begin;

  // We'll discard points on the plan that are outside the local costmap
  const auto& costmap = *costmap_ros_->getCostmap();
  double max_costmap_extent = std::max(costmap.getSizeInMetersX(), costmap.getSizeInMetersY()) / 2.0;

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto& global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    if (!nav_2d_utils::transformPose(tf_buffer_, costmap_ros_->getGlobalFrameID(), stamped_pose, transformed_pose,
                                     transform_tolerance_))
    {
      throw nav2_core::PlannerException("Unable to transform plan pose into local frame");
    }
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;
  std::transform(transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
                 transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  if (transformation_begin != global_plan_poses.begin())
  {
    pruned_plan_.header = global_plan_.header;
    pruned_plan_poses.insert(pruned_plan_poses.end(), std::make_move_iterator(global_plan_poses.begin()),
                             std::make_move_iterator(transformation_begin));
    global_plan_poses.erase(global_plan_poses.begin(), transformation_begin);
  }

  if (transformed_plan.poses.empty())
  {
    RCLCPP_WARN(logger_,
                "No pose of the global plan is inside the local costmap projecting closest pose.");

    const double min_x = costmap.getOriginX();
    const double min_y = costmap.getOriginY();
    const double max_x = min_x + costmap.getSizeInMetersX();
    const double max_y = min_y + costmap.getSizeInMetersY();
    auto clampToBounds = [](double value, double min, double max) {
      return std::max(min, std::min(value, max));
    };

    geometry_msgs::msg::PoseStamped projected_pose = transformGlobalPoseToLocal(transformation_begin_pose);
    projected_pose.pose.position.x = clampToBounds(projected_pose.pose.position.x, min_x, max_x);
    projected_pose.pose.position.y = clampToBounds(projected_pose.pose.position.y, min_y, max_y);
    transformed_plan.poses.push_back(projected_pose);
  }

  return transformed_plan;
}

void PathHandler::setPlan(const nav_msgs::msg::Path& path)
{
  global_plan_ = path;
  pruned_plan_.header = path.header;
  pruned_plan_.poses.clear();
}

void PathHandler::resetPlan()
{
  auto& pruned_plan_poses = pruned_plan_.poses;
  if (pruned_plan_poses.empty())
  {
    return;
  }
  nav_msgs::msg::Path restored_plan;
  restored_plan.header = pruned_plan_.header;
  restored_plan.poses = std::move(pruned_plan_poses);

  auto& global_plan_poses = global_plan_.poses;
  restored_plan.poses.insert(restored_plan.poses.end(), std::make_move_iterator(global_plan_poses.begin()),
                             std::make_move_iterator(global_plan_poses.end()));
  global_plan_poses.clear();
  global_plan_ = std::move(restored_plan);
  pruned_plan_poses.clear();
}


}  // namespace mpc
