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
