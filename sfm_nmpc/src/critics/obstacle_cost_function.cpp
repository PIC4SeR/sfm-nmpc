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


#include "sfm_nmpc/critics/obstacle_cost_function.hpp"
namespace sfm_nmpc
{

ObstacleCost::ObstacleCost(
  double weight, const nav2_costmap_2d::Costmap2D * costmap,
  const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> & costmap_interpolator,
  const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
  unsigned int control_horizon, unsigned int block_length)
: weight_(weight),
  costmap_origin_(costmap->getOriginX(), costmap->getOriginY()),
  costmap_resolution_(costmap->getResolution()),
  costmap_interpolator_(costmap_interpolator)
{
  control_horizon_ = control_horizon;
  robot_init_ = robot_init;
  current_position_ = current_position;
  time_step_ = time_step;
  block_length_ = block_length;
}

}  // namespace sfm_nmpc
