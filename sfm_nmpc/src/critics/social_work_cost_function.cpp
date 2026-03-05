#include "sfm_nmpc/critics/social_work_cost_function.hpp"

namespace sfm_nmpc
{

SocialWorkCost::SocialWorkCost(double weight, const AgentsStates& agents_init,
                               const geometry_msgs::msg::Pose& robot_init, const double counter,
                               unsigned int current_position, double time_step, unsigned int control_horizon,
                               unsigned int block_length)
  : weight_(weight)
  , robot_init_(robot_init)
  , counter_(counter)
  , current_position_(current_position)
  , time_step_(time_step)
  , control_horizon_(control_horizon)
  , block_length_(block_length)
{
  for (unsigned int j = 0; j < agents_init.size(); j++)
  {
    original_agents_.col(j) << agents_init[j][0], agents_init[j][1], agents_init[j][2], agents_init[j][3],
        agents_init[j][4], agents_init[j][5];
  }

  sfm_lambda_ = 2.0;
  sfm_gamma_ = 0.35;
  sfm_nPrime_ = 3.0;
  sfm_n_ = 2.0;
  sfm_relaxationTime_ = 0.5;
  sfm_forceFactorSocial_ = 2.1;
}

}  // namespace sfm_nmpc
