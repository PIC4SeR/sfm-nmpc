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

#ifndef MPC_SFM_MOTION_MODEL__TOOLS__MOTION_MODEL_HPP_
#define MPC_SFM_MOTION_MODEL__TOOLS__MOTION_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"

namespace mpc_sfm_motion_model
{

/**
 * @brief Base interface for robot kinematic models used by the trajectorizer.
 */
class MotionModel
{
public:
  struct Limits
  {
    double max_linear_acc{ 0.0 };
    double max_angular_acc{ 0.0 };
    double max_linear_speed{ 0.0 };
    double max_angular_speed{ 0.0 };
  };

  explicit MotionModel(const Limits& limits)
  : limits_(limits)
  {
  }

  virtual ~MotionModel() = default;

  /**
   * @brief Integrate a planar motion step applying velocity and acceleration limits.
   * @param current_state Robot current state in the world frame (as an odometry message).
   * @param cmd Desired commanded twist expressed in the robot frame.
   * @param dt Integration time step in seconds.
   * @return The state after applying the motion command for the given time step.
   */
  virtual nav_msgs::msg::Odometry integrate(const nav_msgs::msg::Odometry& current_state,
                                             const geometry_msgs::msg::Twist& cmd,
                                             const double dt) const = 0;

  /**
    * @brief Limit the applied command based on acceleration and speed limits
    */
  double clampCommand(
    const double& target, const double& current, const double& limit, const double dt) const
  {
    if (limit <= 0.0)
    {
      return target;
    }
    const double max_delta = limit * dt;
    const double delta = std::clamp(target - current, -max_delta, max_delta);
    return current + delta;
  }
                                             
protected:
  Limits limits_;
};

/**
 * @brief Holonomic model that integrates planar velocities in the robot frame.
 * Linear components are projected to the world using the current heading and
 * angular velocity is applied directly to theta.
 */
class HolonomicMotionModel : public MotionModel
{
public:
  explicit HolonomicMotionModel(const Limits& limits)
  : MotionModel(limits)
  {
  }



  nav_msgs::msg::Odometry integrate(const nav_msgs::msg::Odometry& current_state,
                                       const geometry_msgs::msg::Twist& cmd, double dt) const override
  {
    geometry_msgs::msg::Twist applied = current_state.twist.twist;
    nav_msgs::msg::Odometry new_state = current_state;
    
    applied.linear.x = clampCommand(cmd.linear.x, current_state.twist.twist.linear.x, limits_.max_linear_acc, dt);
    applied.linear.y = clampCommand(cmd.linear.y, current_state.twist.twist.linear.y, limits_.max_linear_acc, dt);
    applied.angular.z = clampCommand(cmd.angular.z, current_state.twist.twist.angular.z, limits_.max_angular_acc, dt);

    const double speed = std::hypot(applied.linear.x, applied.linear.y);
    if (limits_.max_linear_speed > 0.0 && speed > limits_.max_linear_speed)
    {
      const double scale = limits_.max_linear_speed / speed;
      applied.linear.x *= scale;
      applied.linear.y *= scale;
    }

    if (limits_.max_angular_speed > 0.0)
    {
      applied.angular.z = std::clamp(applied.angular.z, -limits_.max_angular_speed, limits_.max_angular_speed);
    }
    double x = current_state.pose.pose.position.x;
    double y = current_state.pose.pose.position.y;
    double theta = tf2::getYaw(current_state.pose.pose.orientation);
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    x += (applied.linear.x * cos_theta - applied.linear.y * sin_theta) * dt;
    y += (applied.linear.x * sin_theta + applied.linear.y * cos_theta) * dt;
    theta += applied.angular.z * dt;
    new_state.header.stamp = (rclcpp::Time(new_state.header.stamp) +
      rclcpp::Duration::from_seconds(dt));
    new_state.pose.pose.position.x = x;
    new_state.pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    new_state.pose.pose.orientation = tf2::toMsg(q);
    new_state.twist.twist = applied;
    return new_state;
  }
};

/**
 * @brief Unicycle model (differential drive) that ignores lateral velocity.
 * The forward velocity is projected using the current heading; angular
 * velocity directly updates theta.
 */
class UnicycleMotionModel : public MotionModel
{
public:
  explicit UnicycleMotionModel(const Limits& limits)
  : MotionModel(limits)
  {
  }

  nav_msgs::msg::Odometry integrate(const nav_msgs::msg::Odometry& current_state,
                                       const geometry_msgs::msg::Twist& cmd, double dt) const override
  {
    geometry_msgs::msg::Twist applied = current_state.twist.twist;
    nav_msgs::msg::Odometry new_state = current_state;
    applied.linear.x = clampCommand(cmd.linear.x, current_state.twist.twist.linear.x, limits_.max_linear_acc, dt);
    applied.linear.y = 0.0;  // lateral motion not allowed in unicycle model
    applied.angular.z = clampCommand(cmd.angular.z, current_state.twist.twist.angular.z, limits_.max_angular_acc, dt);

    if (limits_.max_linear_speed > 0.0)
    {
      applied.linear.x = std::clamp(applied.linear.x, -limits_.max_linear_speed, limits_.max_linear_speed);
    }

    if (limits_.max_angular_speed > 0.0)
    {
      applied.angular.z = std::clamp(applied.angular.z, -limits_.max_angular_speed, limits_.max_angular_speed);
    }

    double x = current_state.pose.pose.position.x;
    double y = current_state.pose.pose.position.y;
    double theta = tf2::getYaw(current_state.pose.pose.orientation);
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    x += applied.linear.x * cos_theta * dt;
    y += applied.linear.x * sin_theta * dt;
    theta += applied.angular.z * dt;

    new_state.header.stamp = (rclcpp::Time(new_state.header.stamp) +
      rclcpp::Duration::from_seconds(dt));
    new_state.pose.pose.position.x = x;
    new_state.pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    new_state.pose.pose.orientation = tf2::toMsg(q);
    new_state.twist.twist = applied;
    return new_state;
  }
};

using PlanarMotionModel = HolonomicMotionModel;  // Backward compatibility alias

}  // namespace mpc_sfm_motion_model

#endif  // MPC_SFM_MOTION_MODEL__TOOLS__MOTION_MODEL_HPP_
