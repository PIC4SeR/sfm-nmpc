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