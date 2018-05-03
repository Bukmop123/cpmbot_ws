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
  actualTime=0;
  _comm_Arduino.data = 0.0;

  std::string hw_topic_command;
  std::string hw_topic__feedback;

  //nh.param("my_num", i, 42);

  nh.param<std::string>("hardware/publisher/detail/command", hw_topic_command, "hardware/publisher/detail/command");
  nh.param<std::string>("hardware/subscriber/detail/feedback", hw_topic__feedback, "hardware/subscriber/detail/feedback");
 

  HardwareInterfacePublisher_ = nh.advertise<cpmbot::cpmbot_detail>(hw_topic_command, 10);

  //controller have their own topic for commands
  HardwareFeedbackSubscriber_ =  nh.subscribe(hw_topic__feedback,10, &CpmBotHWInterface::HardwareFeedbackCallback ,this); 


  //Communication test with the Arduino:
  comm_ArduinoPublisher_ = nh.advertise<std_msgs::Float64>("communication/arduino/command", 10);
  comm_ArduinoSubscriber_ =  nh.subscribe("communication/arduino/feedback",10, &CpmBotHWInterface::HardwareFeedbackCallback ,this); 


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
  joint_velocity_[0] = _hardware_feedback.body_left;
  joint_velocity_[1] = _hardware_feedback.body_right;

  joint_position_[2] = _hardware_feedback.arm_front_left;
  joint_position_[3] = _hardware_feedback.arm_front_right;
  joint_position_[4] = _hardware_feedback.arm_back_left;
  joint_position_[5] = _hardware_feedback.arm_back_right;

  joint_velocity_[6] = _hardware_feedback.mecanum_front_left;
  joint_velocity_[7] = _hardware_feedback.mecanum_front_right;
  joint_velocity_[8] = _hardware_feedback.mecanum_back_left;
  joint_velocity_[9] = _hardware_feedback.mecanum_back_right;
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



      _hardware_cmd.body_left = joint_effort_command_[0];
      _hardware_cmd.body_left = elapsed_time.toSec();


      _hardware_cmd.body_right = joint_effort_command_[1];
      _hardware_cmd.body_right = actualTime;
      
      _hardware_cmd.arm_front_left = joint_effort_command_[2];
      _hardware_cmd.arm_front_right = joint_effort_command_[3];
      _hardware_cmd.arm_back_left = joint_effort_command_[4];
      _hardware_cmd.arm_back_right = joint_effort_command_[5];

      _hardware_cmd.mecanum_front_left = joint_effort_command_[6];
      _hardware_cmd.mecanum_front_right = joint_effort_command_[7];
      _hardware_cmd.mecanum_back_left = joint_effort_command_[8];
      _hardware_cmd.mecanum_back_right = joint_effort_command_[9];


      //SetHardwareInterfacePublish();
      actualTime = actualTime + elapsed_time.toSec();

        if((actualTime) >= 0.01 ){
          SetHardwareInterfacePublish();
          actualTime = 0.0;
        }
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

void CpmBotHWInterface::comm_ArduinoCallback(const std_msgs::Float64::ConstPtr& cmd_msg)
{
  _comm_Arduino.data = cmd_msg->data;
  if(_comm_Arduino.data > 100){
    _comm_Arduino.data = 0.0;
  }

  //comm_ArduinoPublisher_.publish(_comm_Arduino);
}

void CpmBotHWInterface::HardwareFeedbackCallback(const cpmbot::cpmbot_detail::ConstPtr& cmd_msg)
{
  _hardware_feedback.body_left = cmd_msg->body_left;
  _hardware_feedback.body_right = cmd_msg->body_right;

  _hardware_feedback.arm_front_left = cmd_msg->arm_front_left;
  _hardware_feedback.arm_front_right = cmd_msg->arm_front_right;
  _hardware_feedback.arm_back_left = cmd_msg->arm_back_left;
  _hardware_feedback.arm_back_right = cmd_msg->arm_back_right;

  _hardware_feedback.mecanum_front_left = cmd_msg->mecanum_front_left;
  _hardware_feedback.mecanum_front_right = cmd_msg->mecanum_front_right;
  _hardware_feedback.mecanum_back_left = cmd_msg->mecanum_back_left;
  _hardware_feedback.mecanum_back_right = cmd_msg->mecanum_back_right;
}

void CpmBotHWInterface::SetHardwareInterfacePublish()
{
  ros::Time current_time = ros::Time::now();
  _hardware_cmd.header.stamp = current_time;
  _hardware_cmd.header.frame_id = "command robot detail";

  
  comm_ArduinoPublisher_.publish(_comm_Arduino);
  HardwareInterfacePublisher_.publish(_hardware_cmd);

}


}  // namespace
