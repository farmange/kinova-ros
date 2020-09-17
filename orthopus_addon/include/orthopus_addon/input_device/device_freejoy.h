/*
 *  device_freejoy.h
 *  Copyright (C) 2019 Orthopus
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CARTESIAN_CONTROLLER_DEVICE_FREEJOY_H
#define CARTESIAN_CONTROLLER_DEVICE_FREEJOY_H

#include <ros/ros.h>

#include "kinova_msgs/JointVelocity.h"
#include "sensor_msgs/Joy.h"
// #include "std_msgs/Bool.h"
// #include "std_msgs/Int8.h"

namespace input_device
{
class DeviceFreejoy
{
public:
  DeviceFreejoy();

private:
  ros::NodeHandle n_;
  ros::Subscriber device_sub_;
  ros::Publisher cmd_pub_;

  ros::Time debounce_button_lb_;
  ros::Time debounce_button_la_;
  int button_lb_;
  int button_la_;
  double velocity_factor_;

  double debounce_button_time_;
  double velocity_factor_inc_;

  void callbackJoy(const sensor_msgs::Joy::ConstPtr& msg);
  void processButtons(const sensor_msgs::Joy::ConstPtr& msg);
  void debounceButtons(const sensor_msgs::Joy::ConstPtr& msg, const int button_id, ros::Time& debounce_timer_ptr,
                       int& button_value_ptr);
  void updateVelocityFactor();
};
}
#endif
