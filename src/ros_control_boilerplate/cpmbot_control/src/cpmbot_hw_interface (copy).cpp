/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the CpmBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/
     

#include <cpmbot_control/cpmbot_hw_interface.h>

namespace cpmbot_control
{
  

CpmBotHWInterface::CpmBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{


  std::string hw_topic_joint1_cmd;
  std::string hw_topic_joint1_fb;

  //nh.param("my_num", i, 42);
  nh.param("joint1_model_number", joint1_model_number_, 0);

  nh.param<std::string>("hardware/subscriber/detail/feedback", hw_topic_joint1_fb, "hardware/subscriber/detail/feedback");
  nh.param<std::string>("hardware/publisher/detail/command", base_link_left_topic, "hardware/publisher/detail/command");

  //nh.param<std::string>("hardware/publisher/base_link/left_command", base_link_left_topic, "hardware/base/left/command");
  nh.param<std::string>("hardware/publisher/base_link/right_command", base_link_right_topic, "hardware/base/right/command");
  nh.param<std::string>("hardware/publisher/arm/front_left_command", arm_front_left_topic, "hardware/arm/front_left/command");
  nh.param<std::string>("hardware/publisher/arm/front_right_command", arm_front_right_topic, "hardware/arm/front_right/command");
  nh.param<std::string>("hardware/publisher/arm/back_left_command", arm_back_left_topic, "hardware/arm/back_left/command");
  nh.param<std::string>("hardware/publisher/arm/back_right_command", arm_back_right_topic, "hardware/arm/back_right/command");
  nh.param<std::string>("hardware/publisher/mecanum/front_left_command", mecanum_front_left_topic, "hardware/mecanum/front_left/command");
  nh.param<std::string>("hardware/publisher/mecanum/front_right_command", mecanum_front_right_topic, "hardware/mecanum/front_right/command");
  nh.param<std::string>("hardware/publisher/mecanum/back_left_command", mecanum_back_left_topic, "hardware/mecanum/back_left/command");
  nh.param<std::string>("hardware/publisher/mecanum/back_right_command", mecanum_back_right_topic, "hardware/mecanum/back_right/command");


  base_link_leftPublisher_ = nh.advertise<std_msgs::Float64>(base_link_left_topic, 10);
  base_link_rightPublisher_ = nh.advertise<std_msgs::Float64>(base_link_right_topic, 10);
  arm_front_leftPublisher_ = nh.advertise<std_msgs::Float64>(arm_front_left_topic, 10);
  arm_front_rightPublisher_ = nh.advertise<std_msgs::Float64>(arm_front_right_topic, 10);
  arm_back_leftPublisher_ = nh.advertise<std_msgs::Float64>(arm_back_left_topic, 10);
  arm_back_rightPublisher_ = nh.advertise<std_msgs::Float64>(arm_back_right_topic, 10);
  mecanum_front_leftPublisher_ = nh.advertise<std_msgs::Float64>(mecanum_front_left_topic, 10);
  mecanum_front_rightPublisher_ = nh.advertise<std_msgs::Float64>(mecanum_front_right_topic, 10);
  mecanum_back_leftPublisher_ = nh.advertise<std_msgs::Float64>(mecanum_back_left_topic, 10);
  mecanum_back_rightPublisher_ = nh.advertise<std_msgs::Float64>(mecanum_back_right_topic, 10);

  //controller have their own topic for commands
  HardwareJointFeedbackSubscriber_ =  nh.subscribe(hw_topic_joint1_fb,10, &CpmBotHWInterface::HardwareJointFeedbackCallback ,this); 



  ROS_INFO_NAMED("cpmbot_hw_interface", "CpmBotHWInterface Ready.");
}

void CpmBotHWInterface::read(ros::Duration &elapsed_time)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  joint_position_[arm_front_left_mn_] = joint1_feedback_position_;
  joint_position_[arm_front_right_mn_] = joint1_feedback_position_;
  joint_position_[arm_back_left_mn_] = joint1_feedback_position_;
  joint_position_[arm_back_right_mn_] = joint1_feedback_position_;

}

void CpmBotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  //for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  //  joint_position_[joint_id] += joint_position_command_[joint_id];
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------



  joint1_command_effort_.data = joint_position_command_[0];
  //joint1_command_effort_.data = joint_effort_command_[0];

  //HardwareJoint1CommandPublisher_.publish(joint1_command_effort_);
  base_link_leftPublisher_.publish(joint1_command_effort_);
  //base_link_rightPublisher_.publish(joint1_command_effort_);
  //arm_front_leftPublisher_.publish(joint1_command_effort_);
  //arm_front_rightPublisher_.publish(joint1_command_effort_);
  //arm_back_leftPublisher_.publish(joint1_command_effort_);
  //arm_back_rightPublisher_.publish(joint1_command_effort_);
  //mecanum_front_leftPublisher_.publish(joint1_command_effort_);
  //mecanum_front_rightPublisher_.publish(joint1_command_effort_);
  //mecanum_back_leftPublisher_.publish(joint1_command_effort_);
  //mecanum_back_rightPublisher_.publish(joint1_command_effort_);

}

void CpmBotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void CpmBotHWInterface::HardwareJointFeedbackCallback(const std_msgs::Float64::ConstPtr& cmd_msg)
{
  joint1_feedback_position_ = cmd_msg->data;
}


}  // namespace
