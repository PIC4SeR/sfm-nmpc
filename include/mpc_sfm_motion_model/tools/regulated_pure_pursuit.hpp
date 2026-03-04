
#pragma once

#include <algorithm>
#include <cmath>
#include <functional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "angles/angles.h"

#include "tf2/utils.h"

namespace mpc_sfm_motion_model
{

class RegulatedPurePursuit
{
public:
  struct Params
  {
    double goal_dist_tol{ 0.25 };
    double rotate_to_heading_angular_vel{ 0.75 };
    double time_step{ 0.05 };
    double max_angular_accel{ 1.0 };
    double min_approach_linear_velocity{ 0.05 };
    bool use_rotate_to_heading{ true };
    bool use_interpolation{ false };
    double rotate_to_heading_min_angle{ 0.1 };
    bool use_regulated_linear_velocity_scaling_{ true };
    double regulated_linear_scaling_min_radius_{ 0.5 };
    double regulated_linear_scaling_min_speed_{ 0.1 };
    double desired_linear_velocity_{ 0.5 };
    // Cost-regulated linear velocity scaling params
    bool use_cost_regulated_linear_velocity_scaling{ true };
    double cost_scaling_dist{ 0.6 };
    double cost_scaling_gain{ 1.0 };
    double inflation_cost_scaling_factor{ 3.0 };
    // Collision detection params
    bool use_collision_detection{ true };
    double max_allowed_time_to_collision_up_to_carrot{ 1.0 };
    double projection_lookahead_resolution{ 0.1 };
  };

  RegulatedPurePursuit() = default;
  explicit RegulatedPurePursuit(const Params& params) : params_(params) {}

  void updateParams(const Params& params)
  {
    params_ = params;
  }

  void rotateToHeading(
    double& linear_vel, double& angular_vel,
    const double& angle_to_path, const geometry_msgs::msg::Twist& curr_speed) const
  {
    linear_vel = 0.0;
    const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
    angular_vel = sign * params_.rotate_to_heading_angular_vel;

    const double dt = params_.time_step;
    const double min_feasible_angular_speed = curr_speed.angular.z - params_.max_angular_accel * dt;
    const double max_feasible_angular_speed = curr_speed.angular.z + params_.max_angular_accel * dt;
    angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

    const double max_vel_to_stop = std::sqrt(2.0 * params_.max_angular_accel * std::abs(angle_to_path));
    if (std::abs(angular_vel) > max_vel_to_stop)
    {
      angular_vel = sign * max_vel_to_stop;
    }
  }

  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point& center,
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2,
    double r) const
  {
    // Shift points to circle center at origin
    geometry_msgs::msg::Point p1_shifted;
    p1_shifted.x = p1.x - center.x;
    p1_shifted.y = p1.y - center.y;
    geometry_msgs::msg::Point p2_shifted;
    p2_shifted.x = p2.x - center.x;
    p2_shifted.y = p2.y - center.y;
    const double x1 = p1_shifted.x;
    const double x2 = p2_shifted.x;
    const double y1 = p1_shifted.y;
    const double y2 = p2_shifted.y;
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dr2 = dx * dx + dy * dy;
    const double D = x1 * y2 - x2 * y1;

    const double d1 = x1 * x1 + y1 * y1;
    const double d2 = x2 * x2 + y2 * y2;
    const double dd = d2 - d1;

    geometry_msgs::msg::Point p;
    const double sqrt_term = std::sqrt(r * r * dr2 - D * D);
    p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    // Shift back to original center
    p.x += center.x;
    p.y += center.y;
    return p;  
  }

  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double& lookahead_dist,
    const nav_msgs::msg::Path& transformed_plan, const geometry_msgs::msg::PoseStamped& robot_pose) const
  {
    // check if the path and robot pose are in the same frame
    if (transformed_plan.header.frame_id != robot_pose.header.frame_id)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RegulatedPurePursuit"),
        "Transformed plan frame '%s' and robot pose frame '%s' do not match",
        transformed_plan.header.frame_id.c_str(),
        robot_pose.header.frame_id.c_str());
      return geometry_msgs::msg::PoseStamped();
    }

    // Find the first point on the path that is at least lookahead_dist away
    auto goal_pose_it = std::find_if(
      transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto& ps) {
       return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps) >= lookahead_dist;
      });

    // If no such point is found, return the last point in the path
    if (goal_pose_it == transformed_plan.poses.end())
    {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    }
    else if (params_.use_interpolation && goal_pose_it != transformed_plan.poses.begin())
    {
      auto prev_pose_it = std::prev(goal_pose_it);
      auto point = circleSegmentIntersection(
        robot_pose.pose.position,
        prev_pose_it->pose.position,
        goal_pose_it->pose.position, lookahead_dist);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = prev_pose_it->header.frame_id;
      pose.header.stamp = goal_pose_it->header.stamp;
      pose.pose.position = point;
      return pose;
    }

    return *goal_pose_it;
  }

  void applyApproachVelocityScaling(
    const nav_msgs::msg::Path& path, 
    const geometry_msgs::msg::PoseStamped& robot_pose,
    double& linear_vel) const
  {
    const double goal_dist = path.poses.empty() ? 0.0 : nav2_util::geometry_utils::euclidean_distance(
      robot_pose, path.poses.back());
    applyApproachVelocityScaling(goal_dist, linear_vel);
  }

  void applyApproachVelocityScaling(double goal_dist, double& linear_vel) const
  {
    double approach_vel = linear_vel;
    const double velocity_scaling = approachVelocityScalingFactor(goal_dist);
    const double unbounded_vel = approach_vel * velocity_scaling;
    if (unbounded_vel < params_.min_approach_linear_velocity)
    {
      approach_vel = params_.min_approach_linear_velocity;
    }
    else
    {
      approach_vel *= velocity_scaling;
    }

    linear_vel = std::min(linear_vel, approach_vel);
  }

  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::PoseStamped & carrot_pose,
    double & angle_to_path)
  {
    // Whether we should rotate robot to rough path heading
    angle_to_path = atan2(carrot_pose.pose.position.y - robot_pose.pose.position.y, carrot_pose.pose.position.x - robot_pose.pose.position.x) -
                    tf2::getYaw(robot_pose.pose.orientation);
    // Normalize angle to [-pi, pi]
    angle_to_path = angles::normalize_angle(angle_to_path);
    return params_.use_rotate_to_heading && fabs(angle_to_path) > params_.rotate_to_heading_min_angle;
  }

  bool shouldRotateToGoalHeading(
    const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::PoseStamped & carrot_pose)
  {
    // Whether we should rotate robot to goal heading
    double dist_to_goal = nav2_util::geometry_utils::euclidean_distance(
      robot_pose, carrot_pose);
    // If we're close to the goal, check if we need to rotate to the goal heading
    auto angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation) - tf2::getYaw(robot_pose.pose.orientation);
    angle_to_goal = angles::normalize_angle(angle_to_goal);
    return params_.use_rotate_to_heading && dist_to_goal < params_.goal_dist_tol && fabs(angle_to_goal) > params_.rotate_to_heading_min_angle;
  }

  /**
   * @brief Apply constraints to the linear velocity.
   * Overload without pose_cost (backward compatible, no cost scaling).
   */
  void applyConstraints(
    const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & robot_pose,
    double & linear_vel, double & sign)
  {
    applyConstraints(curvature, curr_speed, -1.0, path, robot_pose, linear_vel, sign);
  }

  /**
   * @brief Apply constraints including cost-regulated velocity scaling.
   * @param curvature Curvature of the current arc
   * @param curr_speed Current robot speed
   * @param pose_cost Costmap cost at the robot pose (0-254). Use -1 to skip cost scaling.
   * @param path Current transformed path
   * @param robot_pose Current robot pose
   * @param linear_vel Linear velocity to constrain (in/out)
   * @param sign Direction sign (+1 forward, -1 reverse)
   */
  void applyConstraints(
    const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
    const double & pose_cost,
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & robot_pose,
    double & linear_vel, double & sign)
  {
    double curvature_vel = linear_vel;
    double cost_vel = linear_vel;

    // limit the linear velocity by curvature
    const double radius = fabs(1.0 / curvature);
    const double & min_rad = params_.regulated_linear_scaling_min_radius_;
    if (params_.use_regulated_linear_velocity_scaling_ && radius < min_rad) {
      curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
    }

    // limit the linear velocity by proximity to obstacles
    // Nav2 costmap cost constants: FREE_SPACE=0, NO_INFORMATION=255, INSCRIBED_INFLATED_OBSTACLE=253
    constexpr double NO_INFORMATION = 255.0;
    constexpr double FREE_SPACE = 0.0;
    constexpr double INSCRIBED_INFLATED_OBSTACLE = 253.0;
    if (params_.use_cost_regulated_linear_velocity_scaling &&
      pose_cost >= 0.0 &&
      pose_cost != NO_INFORMATION &&
      pose_cost != FREE_SPACE)
    {
      const double min_distance_to_obstacle = (-1.0 / params_.inflation_cost_scaling_factor) *
        std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1));

      if (min_distance_to_obstacle < params_.cost_scaling_dist) {
        cost_vel *= params_.cost_scaling_gain * min_distance_to_obstacle / params_.cost_scaling_dist;
      }
    }

    // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
    linear_vel = std::min(cost_vel, curvature_vel);
    linear_vel = std::max(linear_vel, params_.regulated_linear_scaling_min_speed_);

    applyApproachVelocityScaling(path, robot_pose, linear_vel);

    // Limit linear velocities to be valid
    linear_vel = std::clamp(fabs(linear_vel), 0.0, params_.desired_linear_velocity_);
    linear_vel = sign * linear_vel;
  }

  /**
   * @brief Check whether collision is imminent by forward-projecting the arc.
   *
   * Uses the same approach as Nav2 RPP: projects the robot along its current
   * commanded arc (linear_vel, angular_vel) in small time steps and checks each
   * projected pose for collision up to the carrot distance or the max allowed
   * time horizon.
   *
   * @param robot_pose  Current robot pose (in odom / global frame)
   * @param linear_vel  Commanded linear velocity
   * @param angular_vel Commanded angular velocity
   * @param carrot_dist Distance to the lookahead (carrot) point
   * @param collision_checker A callable bool(double x, double y, double theta)
   *        that returns true if the given pose is in collision.
   *        Typically wraps a costmap footprint collision check.
   * @return true if a collision is detected along the projected arc
   */
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const double & linear_vel, const double & angular_vel,
    const double & carrot_dist,
    const std::function<bool(double, double, double)> & collision_checker) const
  {
    if (!params_.use_collision_detection) {
      return false;
    }

    // Check current pose first
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    if (collision_checker(
        robot_pose.pose.position.x,
        robot_pose.pose.position.y,
        robot_yaw))
    {
      return true;
    }

    // Determine projection time step
    double projection_time = params_.projection_lookahead_resolution;
    if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) {
      // Rotating in place - use angular-based projection step
      projection_time = 0.1 / fabs(angular_vel);  // ~0.1 rad per step
    } else if (fabs(linear_vel) >= 0.01) {
      // Normal path tracking - project per resolution step
      projection_time = params_.projection_lookahead_resolution / fabs(linear_vel);
    } else {
      // Robot is essentially stopped, no collision ahead
      return false;
    }

    // Forward-simulate the arc
    geometry_msgs::msg::Pose2D curr_pose;
    curr_pose.x = robot_pose.pose.position.x;
    curr_pose.y = robot_pose.pose.position.y;
    curr_pose.theta = robot_yaw;

    int i = 1;
    while (i * projection_time < params_.max_allowed_time_to_collision_up_to_carrot) {
      i++;

      // Propagate pose along the arc
      curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
      curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
      curr_pose.theta += projection_time * angular_vel;

      // Stop checking beyond the carrot distance
      if (std::hypot(
          curr_pose.x - robot_pose.pose.position.x,
          curr_pose.y - robot_pose.pose.position.y) > carrot_dist)
      {
        break;
      }

      // Check for collision at projected pose
      if (collision_checker(curr_pose.x, curr_pose.y, curr_pose.theta)) {
        return true;
      }
    }

    return false;
  }

private:
  double approachVelocityScalingFactor(const double goal_dist) const
  {
    if (goal_dist <= params_.goal_dist_tol)
    {
      return 0.0;
    }

    const double max_dist = std::max(params_.goal_dist_tol * 4.0, params_.goal_dist_tol + 1e-3);
    const double scale = goal_dist / max_dist;
    return std::clamp(scale, 0.0, 1.0);
  }

  Params params_;
};

}  // namespace mpc_sfm_motion_model