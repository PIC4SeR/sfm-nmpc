#include "sfm_nmpc/critics/velocity_feasibility_cost_function.hpp"

namespace sfm_nmpc
{

VelocityFeasibilityCost::VelocityFeasibilityCost(
  double weight, unsigned int current_position, unsigned int control_horizon)
: weight_(weight)
{
  control_horizon_ = control_horizon;
  current_position_ = current_position;
}

}  // namespace sfm_nmpc