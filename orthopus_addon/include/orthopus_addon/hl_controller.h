/*
 *  hl_controller.h
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
#ifndef HL_CONTROLLER_H
#define HL_CONTROLLER_H

#include <ros/ros.h>

#include "kinova_msgs/JointAngles.h"
#include "kinova_msgs/JointVelocity.h"

namespace orthopus_addon
{
class HLController
{
public:
  HLController();

private:
  ros::NodeHandle n_;
  ros::Subscriber joint_angles_sub_;
  ros::Subscriber joy_cmd_sub_;
  ros::Subscriber gui_cmd_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher upper_limit_pub_;
  ros::Publisher lower_limit_pub_;
  ros::Publisher joint_angle_pub_;
  ros::ServiceServer set_upper_limit_service_;
  ros::ServiceServer set_lower_limit_service_;
  ros::ServiceServer reset_upper_limit_service_;
  ros::ServiceServer reset_lower_limit_service_;

  int sampling_freq_;
  int direction_;
  double joint_max_speed_;
  double sampling_period_;

  kinova_msgs::JointAngles joint_angles_;
  kinova_msgs::JointAngles upper_limit_;
  kinova_msgs::JointAngles lower_limit_;
  kinova_msgs::JointVelocity joy_cmd_;
  kinova_msgs::JointVelocity gui_cmd_;

  void initializeSubscribers_();
  void initializePublishers_();
  void initializeServices_();
  void retrieveParameters_();
  void init_();

  void handleLimits_(kinova_msgs::JointVelocity& cmd, const float divisor);

  void updateJointAngles_(kinova_msgs::JointAngles& joint_angles, int joint_number, float joint_value);
  void setUpperLimit_(const int joint_number, const float upper_limit_value);
  void setLowerLimit_(const int joint_number, const float upper_limit_value);

  void callbackJointAngles_(const kinova_msgs::JointAnglesPtr& msg);
  void callbackJoyCmd_(const kinova_msgs::JointVelocityPtr& msg);
  void callbackGuiCmd_(const kinova_msgs::JointVelocityPtr& msg);
  bool callbackSetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackSetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackResetUpperLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool callbackResetLowerLimit_(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
};
}
#endif
