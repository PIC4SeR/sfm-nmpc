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


#include "sfm_nmpc/critics/agent_angle_cost_function.hpp"

namespace sfm_nmpc
{

AgentAngleCost::AgentAngleCost(double weight, double velocity_alignment_weight, const AgentsStates& agents_init,
                               const geometry_msgs::msg::Pose& robot_init, unsigned int current_position,
                               double time_step, unsigned int control_horizon, unsigned int block_length)
  : weight_(weight)
  , velocity_alignment_weight_(velocity_alignment_weight)
  , agents_init_(agents_init)
  , robot_init_(robot_init)
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
  safe_distance_squared_ = 4.0;
}

}  // namespace sfm_nmpc