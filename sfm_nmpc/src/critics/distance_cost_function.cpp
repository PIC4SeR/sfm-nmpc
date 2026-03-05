#include "sfm_nmpc/critics/distance_cost_function.hpp"
namespace sfm_nmpc
{

DistanceCost::DistanceCost(
  double weight, const Eigen::Matrix<double, 2, 1> point,
  const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
  unsigned int control_horizon, unsigned int block_length)
: weight_(weight), point_(point)
{
  block_length_ = block_length;
  robot_init_ = robot_init;
  current_position_ = current_position;
  time_step_ = time_step;
  control_horizon_ = control_horizon;
}

}  // namespace sfm_nmpc