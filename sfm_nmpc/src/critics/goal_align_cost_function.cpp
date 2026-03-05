#include "sfm_nmpc/critics/goal_align_cost_function.hpp"

namespace sfm_nmpc
{

GoalAlignCost::GoalAlignCost(
  double weight, const Eigen::Matrix<double, 2, 1> goal_heading,
  const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
  unsigned int control_horizon, unsigned int block_length)
: weight_(weight),
  goal_heading_(goal_heading),
  robot_init_(robot_init),
  current_position_(current_position),
  time_step_(time_step),
  control_horizon_(control_horizon),
  block_length_(block_length)
{
}

}  // namespace sfm_nmpc
