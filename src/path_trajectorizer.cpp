// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University
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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mpc_sfm_motion_model/path_trajectorizer.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace mpc_sfm_motion_model
{

PathTrajectorizer::PathTrajectorizer()
{
}
PathTrajectorizer::~PathTrajectorizer()
{
}

int PathTrajectorizer::findWaypointIndex(const nav_msgs::msg::Path& path, double rx, double ry, double target_dist) const
{
  if (path.poses.empty())
  {
    return 0;
  }

  double min_dist = std::numeric_limits<double>::max();
  int wp_index = static_cast<int>(path.poses.size() - 1);
  for (std::size_t idx = path.poses.size(); idx-- > 0;)
  {
    const auto& pose = path.poses[idx].pose;
    const double dx = pose.position.x - rx;
    const double dy = pose.position.y - ry;
    const double dist = std::hypot(dx, dy);
    if (dist <= target_dist)
    {
      wp_index = static_cast<int>(idx);
      break;
    }
    if (dist < min_dist)
    {
      min_dist = dist;
      wp_index = static_cast<int>(idx);
    }
  }

  return wp_index;
}

rclcpp::Time PathTrajectorizer::applyMotionModel(geometry_msgs::msg::PoseStamped& current_rp,
                                                 const geometry_msgs::msg::Twist& cmd,
                                                 const geometry_msgs::msg::Twist& prev_cmd,
                                                 geometry_msgs::msg::Twist& applied_cmd
                                                 ) const
{
  if (!motion_model_)
  {
    RCLCPP_ERROR(logger_, "Motion model not initialized!");
    throw std::runtime_error("Motion model not initialized");
  }
  nav_msgs::msg::Odometry current_state;
  current_state.header = current_rp.header;
  current_state.pose.pose = current_rp.pose;
  current_state.twist.twist = prev_cmd;
  nav_msgs::msg::Odometry new_state = motion_model_->integrate(current_state, cmd, time_step_);
  // update robot pose
  current_rp.pose = new_state.pose.pose;
  applied_cmd = new_state.twist.twist;
  current_rp.header = new_state.header;

  return rclcpp::Time(new_state.header.stamp);
}

void PathTrajectorizer::configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, std::string name,
                                  std::shared_ptr<tf2_ros::Buffer> tf)
{
  auto node = parent.lock();
  clock_ = node->get_clock();
  tf_ = tf;
  plugin_name_ = name + ".trajectorizer";
  logger_ = node->get_logger();

  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".base_frame", rclcpp::ParameterValue("base_footprint"));
  declare_parameter_if_not_declared(node, plugin_name_ + ".time_step", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_time", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".waypoint_dist_tol", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, name + ".max_angular_vel", rclcpp::ParameterValue(1.4));
  declare_parameter_if_not_declared(node, name + ".max_linear_vel", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".motion_model_type", rclcpp::ParameterValue(std::string("unicycle")));
  declare_parameter_if_not_declared(node, plugin_name_ + ".allow_reverse", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".reverse_heading_threshold", rclcpp::ParameterValue(M_PI_2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_reverse_speed", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.75));
  declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(1.0));
  // Obstacle avoidance parameters
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_collision_detection", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".projection_lookahead_resolution", rclcpp::ParameterValue(0.1));
  
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(name + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(name + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".base_frame", base_frame_);
  node->get_parameter(plugin_name_ + ".time_step", time_step_);
  node->get_parameter(plugin_name_ + ".waypoint_dist_tol", waypoint_dist_tol_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".allow_reverse", allow_reverse_);
  node->get_parameter(plugin_name_ + ".reverse_heading_threshold", reverse_heading_threshold_);
  node->get_parameter(plugin_name_ + ".max_reverse_speed", max_reverse_speed_);
  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  // Obstacle avoidance parameters
  bool use_cost_regulated_linear_velocity_scaling{true};
  double cost_scaling_dist{0.6};
  double cost_scaling_gain{1.0};
  double inflation_cost_scaling_factor{3.0};
  bool use_collision_detection{true};
  double max_allowed_time_to_collision_up_to_carrot{1.0};
  double projection_lookahead_resolution{0.1};
  node->get_parameter(plugin_name_ + ".use_cost_regulated_linear_velocity_scaling", use_cost_regulated_linear_velocity_scaling);
  node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist);
  node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain);
  node->get_parameter(plugin_name_ + ".inflation_cost_scaling_factor", inflation_cost_scaling_factor);
  node->get_parameter(plugin_name_ + ".use_collision_detection", use_collision_detection);
  node->get_parameter(plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot", max_allowed_time_to_collision_up_to_carrot);
  node->get_parameter(plugin_name_ + ".projection_lookahead_resolution", projection_lookahead_resolution);
  std::string motion_model_type;
  node->get_parameter(plugin_name_ + ".motion_model_type", motion_model_type);

  max_linear_accel_ = std::max(0.0, max_linear_accel_);
  max_angular_accel_ = std::max(0.0, max_angular_accel_);
  reverse_heading_threshold_ = std::clamp(reverse_heading_threshold_, 0.0, M_PI);
  max_reverse_speed_ = std::clamp(std::abs(max_reverse_speed_), 0.0, desired_linear_vel_);
  min_approach_linear_velocity_ = std::max(0.0, min_approach_linear_velocity_);
  have_last_cmd_ = false;
  last_cmd_ = geometry_msgs::msg::Twist();

  RegulatedPurePursuit::Params pursuit_params;
  pursuit_params.goal_dist_tol = waypoint_dist_tol_;
  pursuit_params.rotate_to_heading_angular_vel = rotate_to_heading_angular_vel_;
  pursuit_params.rotate_to_heading_min_angle = rotate_to_heading_min_angle_;
  pursuit_params.time_step = time_step_;
  pursuit_params.max_angular_accel = max_angular_accel_;
  pursuit_params.min_approach_linear_velocity = min_approach_linear_velocity_;
  pursuit_params.use_rotate_to_heading = use_rotate_to_heading_;
  pursuit_params.use_interpolation = use_interpolation_;
  pursuit_params.desired_linear_velocity_ = desired_linear_vel_;
  // Obstacle avoidance params
  pursuit_params.use_cost_regulated_linear_velocity_scaling = use_cost_regulated_linear_velocity_scaling;
  pursuit_params.cost_scaling_dist = cost_scaling_dist;
  pursuit_params.cost_scaling_gain = cost_scaling_gain;
  pursuit_params.inflation_cost_scaling_factor = inflation_cost_scaling_factor;
  pursuit_params.use_collision_detection = use_collision_detection;
  pursuit_params.max_allowed_time_to_collision_up_to_carrot = max_allowed_time_to_collision_up_to_carrot;
  pursuit_params.projection_lookahead_resolution = projection_lookahead_resolution;
  pure_pursuit_.updateParams(pursuit_params);

  // Choose motion model plugin
  MotionModel::Limits limits{ max_linear_accel_, max_angular_accel_, max_linear_vel_, max_angular_vel_};
  if (motion_model_type == "unicycle")
  {
    motion_model_ = std::make_unique<UnicycleMotionModel>(limits);
    RCLCPP_INFO(logger_, "Motion model set to unicycle");
  }
  else
  {
    motion_model_ = std::make_unique<HolonomicMotionModel>(limits);
    if (motion_model_type != "holonomic")
    {
      RCLCPP_WARN(logger_, "Unknown motion_model_type '%s', defaulting to holonomic", motion_model_type.c_str());
    }
    else
    {
      RCLCPP_INFO(logger_, "Motion model set to holonomic");
    }
  }

  double max_time;
  node->get_parameter(plugin_name_ + ".max_time", max_time);

  RCLCPP_DEBUG(logger_, "-------------------------------------");
  RCLCPP_DEBUG(logger_, "Path Trajectorizer params:");
  RCLCPP_DEBUG(logger_, "desired_linear_vel: %.2f m/s", desired_linear_vel_);
  RCLCPP_DEBUG(logger_, "lookahead_dist: %.2f m", lookahead_dist_);
  RCLCPP_DEBUG(logger_, "max_angular_vel: %.2f rad/s", max_angular_vel_);
  RCLCPP_DEBUG(logger_, "max_linear_vel: %.2f m/s", max_linear_vel_);
  RCLCPP_DEBUG(logger_, "time_step: %.2f secs", time_step_);
  RCLCPP_DEBUG(logger_, "max_time: %.2f secs", max_time);
  RCLCPP_DEBUG(logger_, "base_frame: %s", base_frame_.c_str());
  RCLCPP_DEBUG(logger_, "-------------------------------------");

  max_steps_ = (int)round(max_time / time_step_);

  received_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  computed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("trajectorized_global_plan", 1);
  lookahead_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("lookahead_points", 1);
}

void PathTrajectorizer::cleanup()
{
  RCLCPP_INFO(logger_,
              "Cleaning up path trajectorizer: %s of type"
              " nav2_path_trajectorizer::PathTrajectorizer",
              plugin_name_.c_str());
  received_path_pub_.reset();
  computed_path_pub_.reset();
  lookahead_marker_pub_.reset();
  resetLastCommand();
}

void PathTrajectorizer::activate()
{
  RCLCPP_INFO(logger_,
              "Activating smoother: %s of type "
              "nav2_path_trajectorizer::PathTrajectorizer",
              plugin_name_.c_str());
  received_path_pub_->on_activate();
  computed_path_pub_->on_activate();
  if (lookahead_marker_pub_) {
    lookahead_marker_pub_->on_activate();
  }
}

void PathTrajectorizer::deactivate()
{
  RCLCPP_INFO(logger_,
              "Deactivating smoother: %s of type "
              "nav2_path_trajectorizer::PathTrajectorizer",
              plugin_name_.c_str());
  received_path_pub_->on_deactivate();
  computed_path_pub_->on_deactivate();
  if (lookahead_marker_pub_) {
    lookahead_marker_pub_->on_deactivate();
  }
  resetLastCommand();
}

bool PathTrajectorizer::trajectorize(nav_msgs::msg::Path& path,
  const geometry_msgs::msg::PoseStamped& path_robot_pose, const geometry_msgs::msg::Twist& speed,
  std::vector<geometry_msgs::msg::TwistStamped>& cmds, nav2_core::GoalChecker* goal_checker)
{
  if (path.poses.empty())
  {
    RCLCPP_WARN(logger_, "Received empty path, cannot trajectorize");
    return false;
  }

  // path_robot_pose must be in the same frame as the path
  geometry_msgs::msg::PoseStamped current_rp = path_robot_pose; // to store the current robot pose in the path frame
  geometry_msgs::msg::Twist previous_cmd = speed;

  
  nav_msgs::msg::Path new_path;
  new_path.header.frame_id = path.header.frame_id;
  new_path.header.stamp = current_rp.header.stamp;
  new_path.poses.push_back(current_rp);
  std::vector<geometry_msgs::msg::Point> lookahead_points;
  
  cmds.clear();
  cmds.reserve(max_steps_);
  lookahead_points.reserve( max_steps_);

  // --- Main trajectorization loop ---

  double goal_tolerance = waypoint_dist_tol_;
  if (goal_checker != nullptr)
  {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker->getTolerances(pose_tolerance, velocity_tolerance);
    const double checker_tol = pose_tolerance.position.x;
    if (std::isfinite(checker_tol) && checker_tol > 0.0)
    {
      goal_tolerance = checker_tol;
    }
  }

  RCLCPP_DEBUG(logger_, "Goal tolerance for trajectorization: %.3f m", goal_tolerance);

  // --- 1 --- The goal is the last point in the path
  
  auto goal_distance = nav2_util::geometry_utils::euclidean_distance(
    current_rp, path.poses.back());
  // goal_dist is updated inside the loop
  for (size_t step = 0; step < max_steps_ && goal_distance > goal_tolerance; ++step)
  {
    
    // --- Find the look-ahead point ---
    geometry_msgs::msg::PoseStamped lookahead_pose = pure_pursuit_.getLookAheadPoint(
      lookahead_dist_, path, current_rp);
      lookahead_points.push_back(lookahead_pose.pose.position);
      
    double & wpx = lookahead_pose.pose.position.x;
    double & wpy = lookahead_pose.pose.position.y;
    
  
    double & curr_rx = current_rp.pose.position.x;
    double & curr_ry = current_rp.pose.position.y;
    double curr_rtheta = tf2::getYaw(current_rp.pose.orientation);
    // --- 2 ---
    // Transform way-point into local robot frame and get desired x,y,theta
    double dx = (wpx - curr_rx) * cos(curr_rtheta) + (wpy - curr_ry) * sin(curr_rtheta);
    double dy = -(wpx - curr_rx) * sin(curr_rtheta) + (wpy - curr_ry) * cos(curr_rtheta);
    double curvature = 0.0;
    double wp_dist2 = (dx * dx + dy * dy);

    if (wp_dist2 > 1e-3)
    {
      curvature = 2.0 * dy / (wp_dist2);
    }

    
    double sign = 1.0;
    if (allow_reverse_)
    {
      sign = dx >= 0.0 ? 1.0 : -1.0;
    }

    double linear_vel = desired_linear_vel_;
    double angular_vel = 0.0;
    // Make sure we're in compliance with basic constraints
    
    double angle_to_heading;
    if (pure_pursuit_.shouldRotateToGoalHeading(current_rp, lookahead_pose)) {
      double angle_to_goal = tf2::getYaw(path.poses.back().pose.orientation) - curr_rtheta;
      pure_pursuit_.rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
    } else if (pure_pursuit_.shouldRotateToPath(current_rp, lookahead_pose, angle_to_heading)) {
      pure_pursuit_.rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
    } else {
      pure_pursuit_.applyConstraints(
        curvature, speed,
        path, current_rp,
        linear_vel, sign);

      // Apply curvature to angular velocity after constraining linear velocity
      angular_vel = linear_vel * curvature;
    }

    // --- 3 ---
    geometry_msgs::msg::Twist desired_cmd;
    desired_cmd.linear.x = linear_vel;
    desired_cmd.linear.y = 0.0;
    desired_cmd.angular.z = angular_vel;
    geometry_msgs::msg::Twist applied_cmd;
    
    rclcpp::Time curr_t = applyMotionModel(
      current_rp,
      desired_cmd,
      previous_cmd,
      applied_cmd);
    previous_cmd = applied_cmd;
    new_path.poses.push_back(current_rp);

    geometry_msgs::msg::TwistStamped vel;
    vel.header.frame_id = base_frame_;
    vel.header.stamp = curr_t;
    vel.twist = applied_cmd;
    cmds.push_back(vel);

    goal_distance = nav2_util::geometry_utils::euclidean_distance(
      current_rp, path.poses.back());
  }

  
  // Publish the path received
  received_path_pub_->publish(path);

  // copy the new path into the path
  path.poses.clear();
  path.poses = new_path.poses;

  // publish the new path
  computed_path_pub_->publish(path);
  if (lookahead_marker_pub_ && !lookahead_points.empty() &&
      lookahead_marker_pub_->get_subscription_count() > 0)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header = new_path.header;
    marker.ns = "lookahead_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 0.2;
    marker.points = lookahead_points;
    marker.pose.orientation.w = 1.0;
    marker_array.markers.emplace_back(std::move(marker));
    lookahead_marker_pub_->publish(std::move(marker_array));
  }
  return true;
}

}  // namespace mpc_sfm_motion_model
