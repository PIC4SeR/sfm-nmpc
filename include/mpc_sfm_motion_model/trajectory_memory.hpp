#ifndef MPC_SFM_MOTION_MODEL__TRAJECTORY_MEMORY_HPP_
#define MPC_SFM_MOTION_MODEL__TRAJECTORY_MEMORY_HPP_

#include <math.h>

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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mpc_sfm_motion_model/sfm.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "people_msgs/msg/people.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// This singleton class is used to store the previous trajectory and commands, in order to give a soft start to the
// optimizer Still needs to be tested, but it should work
namespace mpc_sfm_motion_model
{
class TrajectoryMemory
{
public:
  static TrajectoryMemory& getInstance()
  {
    static TrajectoryMemory instance;
    return instance;
  }

  nav_msgs::msg::Path previous_path;
  std::vector<geometry_msgs::msg::TwistStamped> previous_cmds;
  // bool is_initialized = false;

private:
  TrajectoryMemory()
  {
  }  // Private constructor
};
}  // namespace mpc_sfm_motion_model
#endif  // MPC_SFM_MOTION_MODEL__TRAJECTORY_MEMORY_HPP_