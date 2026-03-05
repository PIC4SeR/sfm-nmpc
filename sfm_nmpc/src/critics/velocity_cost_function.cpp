#include "sfm_nmpc/critics/velocity_cost_function.hpp"
namespace sfm_nmpc
{

VelocityCost::VelocityCost(
  double weight, double desired_linear_vel, unsigned int current_position,
  unsigned int control_horizon, unsigned int block_length)
: weight_(weight),
  desired_linear_vel_(desired_linear_vel),
  current_position_(current_position),
  control_horizon_(control_horizon),
  block_length_(block_length)
{
}

}  // namespace sfm_nmpc
