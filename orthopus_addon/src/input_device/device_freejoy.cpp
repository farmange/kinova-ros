/*
 *  device_freejoy.cpp
 *  Copyright (C) 2020 Orthopus
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
#include "ros/ros.h"

#include <orthopus_addon/input_device/freejoy_config.h>
#include "orthopus_addon/input_device/device_freejoy.h"

namespace input_device
{
DeviceFreejoy::DeviceFreejoy()
{
  device_sub_ = n_.subscribe("joy", 1, &DeviceFreejoy::callbackJoy, this);

  cmd_pub_ = n_.advertise<kinova_msgs::JointVelocity>("/j2n6s300_driver/in/joint_velocity", 1);

  debounce_button_lb_ = ros::Time::now();
  debounce_button_la_ = ros::Time::now();

  button_lb_ = 0;
  button_la_ = 0;

  velocity_factor_ = 0.0;

  // ros::param::get("~debounce_button_time", debounce_button_time_);
  // ros::param::get("~velocity_factor_inc", velocity_factor_inc_);
  debounce_button_time_ = 0.2;
  velocity_factor_inc_ = 1;

  ros::spin();
}

void DeviceFreejoy::callbackJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
  processButtons(msg);
  updateVelocityFactor();
  // rostopic pub -
  //     r 100 / j2n6s300_driver / in / joint_velocity kinova_msgs / JointVelocity "{joint1: -15.0, joint2: "
  //                                                                               "0.0, joint3: 0.0, joint4: "
  //                                                                               "0.0, joint5: 0.0, joint6: "

  //                                                                               "0.0}"
  kinova_msgs::JointVelocity joint_vel;
  joint_vel.joint1 = velocity_factor_ * msg->axes[FREEJOY_AXIS_VERTICAL_ARROW];
  cmd_pub_.publish(joint_vel);
}

void DeviceFreejoy::processButtons(const sensor_msgs::Joy::ConstPtr& msg)
{
  button_lb_ = 0;
  button_la_ = 0;

  debounceButtons(msg, FREEJOY_BUTTON_LB, debounce_button_lb_, button_lb_);
  debounceButtons(msg, FREEJOY_BUTTON_LA, debounce_button_la_, button_la_);
}

void DeviceFreejoy::debounceButtons(const sensor_msgs::Joy::ConstPtr& msg, const int button_id,
                                    ros::Time& debounce_timer_ptr, int& button_value_ptr)
{
  if (msg->buttons[button_id])
  {
    if (ros::Time::now() > debounce_timer_ptr)
    {
      debounce_timer_ptr = ros::Time::now() + ros::Duration(debounce_button_time_);
      button_value_ptr = msg->buttons[button_id];
    }
  }
}

void DeviceFreejoy::updateVelocityFactor()
{
  if (button_la_)
  {
    velocity_factor_ += velocity_factor_inc_;
  }
  if (button_lb_)
  {
    velocity_factor_ -= velocity_factor_inc_;
  }
  velocity_factor_ = std::min(velocity_factor_, 150.0);
  velocity_factor_ = std::max(velocity_factor_, 0.0);
}
}

using namespace input_device;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "device_freejoy");

  DeviceFreejoy device_freejoy;

  return 0;
}
