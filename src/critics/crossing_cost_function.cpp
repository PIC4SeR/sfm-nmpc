// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University and PIC4SeR - Politecnico di Torino
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sfm_nmpc/critics/crossing_cost_function.hpp"

namespace sfm_nmpc
{

CrossingCost::CrossingCost(double weight, double bearing_weight,
                           const AgentsStates& agents_init,
                           const geometry_msgs::msg::Pose& robot_init,
                           unsigned int current_position, double time_step,
                           unsigned int control_horizon, unsigned int block_length)
  : weight_(weight)
  , bearing_weight_(bearing_weight)
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
  safe_distance_squared_ = 16.0;  // 6 m — react early enough to steer behind
}

}  // namespace sfm_nmpc
