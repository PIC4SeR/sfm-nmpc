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


#include "sfm_nmpc/people_interface.hpp"

namespace sfm_nmpc
{

PeopleInterface::PeopleInterface(rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
  auto node = parent.lock();
  people_sub_ = node->create_subscription<people_msgs::msg::People>(
    "people", rclcpp::SensorDataQoS(),
    std::bind(&PeopleInterface::people_callback, this, std::placeholders::_1));
}

PeopleInterface::~PeopleInterface() {}

void PeopleInterface::people_callback(const people_msgs::msg::People::SharedPtr people)
{
  mutex_.lock();
  people_ = *people;
  mutex_.unlock();
}

people_msgs::msg::People PeopleInterface::getPeople()
{
  mutex_.lock();
  people_msgs::msg::People p = people_;
  mutex_.unlock();
  return p;
}

}  // namespace sfm_nmpc
