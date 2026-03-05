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


#ifndef MPC_SFM_MOTION_MODEL__PEOPLE_HPP_
#define MPC_SFM_MOTION_MODEL__PEOPLE_HPP_

#include <mutex>

#include "people_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace sfm_nmpc
{

class PeopleInterface
{
public:
  PeopleInterface(rclcpp_lifecycle::LifecycleNode::WeakPtr parent);
  ~PeopleInterface();

  people_msgs::msg::People getPeople();

  void people_callback(const people_msgs::msg::People::SharedPtr people);

private:
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  people_msgs::msg::People people_;
  std::mutex mutex_;
};
}  // namespace sfm_nmpc

#endif