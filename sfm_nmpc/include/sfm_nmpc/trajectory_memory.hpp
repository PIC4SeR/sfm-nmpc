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
#include "sfm_nmpc/sfm.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "people_msgs/msg/people.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// This singleton class is used to store the previous trajectory and commands, in order to give a soft start to the
// optimizer Still needs to be tested, but it should work
namespace sfm_nmpc
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
}  // namespace sfm_nmpc
#endif  // MPC_SFM_MOTION_MODEL__TRAJECTORY_MEMORY_HPP_