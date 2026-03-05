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


#ifndef MPC_SFM_MOTION_MODEL__TYPE_DEFINITIONS_HPP_
#define MPC_SFM_MOTION_MODEL__TYPE_DEFINITIONS_HPP_

#include <Eigen/Core>

typedef Eigen::Matrix<double, 6, 1> AgentStatus;       // x, y, yaw, timestamp, lv, av
typedef std::vector<AgentStatus> AgentsStates;         // vector of agent status (different agents at the same time)
typedef std::vector<AgentStatus> AgentTrajectory;      // vector of agent status (for a single agent trajectory)
typedef std::vector<AgentsStates> AgentsTrajectories;  // vector of agent states (trajectories for all agents)

#endif  // MPC_SFM_MOTION_MODEL__TYPE_DEFINITIONS_HPP_