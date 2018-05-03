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
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef CPMBOT_CONTROL__CPMBOT_HW_INTERFACE_H
#define CPMBOT_CONTROL__CPMBOT_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>

#include "std_msgs/String.h" 
#include "std_msgs/Float64.h"
#include <cpmbot/cpmbot_detail.h>

namespace cpmbot_control
{

/// \brief Hardware interface for a robot
class CpmBotHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  CpmBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);


private:
//-----------------------------for delete--------------------
    //Variables for joint1 
    int joint1_model_number_;
    ros::Publisher HardwareJoint1CommandPublisher_;
    ros::Subscriber HardwareJoint1FeedbackSubscriber_;
    std_msgs::Float64 joint1_command_effort_;
    double joint1_feedback_position_;
//-----------------------------------------------

    ros::Subscriber HardwareJointFeedbackSubscriber_;
    void HardwareJointFeedbackCallback(const std_msgs::Float64::ConstPtr& cmd_msg);

    //Variables for base_link_left
    std::string base_link_left_topic; 
    int base_link_left_mn_;
    ros::Publisher base_link_leftPublisher_;
    std_msgs::Float64 base_link_left_cmd_;
    double base_link_left_fbk_;
    //Variables for base_link_right 
    std::string base_link_right_topic; 
    int base_link_right_mn_;
    ros::Publisher base_link_rightPublisher_;
    std_msgs::Float64 base_link_right_cmd_;
    double base_link_right_fbk_;
    //Variables for arm_front_left 
    std::string arm_front_left_topic;
    int arm_front_left_mn_;
    ros::Publisher arm_front_leftPublisher_;
    std_msgs::Float64 arm_front_left_cmd_;
    double arm_front_left_fbk_;
    //Variables for arm_front_right 
    std::string arm_front_right_topic;
    int arm_front_right_mn_;
    ros::Publisher arm_front_rightPublisher_;
    std_msgs::Float64 arm_front_right_cmd_;
    double arm_front_right_fbk_;
    //Variables for arm_back_left
    std::string arm_back_left_topic;
    int arm_back_left_mn_;
    ros::Publisher arm_back_leftPublisher_;
    std_msgs::Float64 arm_back_left_cmd_;
    double arm_back_left_fbk_;
    //Variables for arm_back_right
    std::string arm_back_right_topic;
    int arm_back_right_mn_;
    ros::Publisher arm_back_rightPublisher_;
    std_msgs::Float64 arm_back_right_cmd_;
    double arm_back_right_fbk_;
    //Variables for mecanum_front_left 
    std::string mecanum_front_left_topic;
    int mecanum_front_left_mn_;
    ros::Publisher mecanum_front_leftPublisher_;
    std_msgs::Float64 mecanum_front_left_cmd_;
    double mecanum_front_left_fbk_;
    //Variables for mecanum_front_right 
    std::string mecanum_front_right_topic;
    int mecanum_front_right_mn_;
    ros::Publisher mecanum_front_rightPublisher_;
    std_msgs::Float64 mecanum_front_right_cmd_;
    double mecanum_front_right_fbk_;
    //Variables for arm_back_left 
    std::string mecanum_back_left_topic;
    int mecanum_back_left_mn_;
    ros::Publisher mecanum_back_leftPublisher_;
    std_msgs::Float64 mecanum_back_left_cmd_;
    double mecanum_back_left_fbk_;
    //Variables for arm_back_right 
    std::string mecanum_back_right_topic;
    int mecanum_back_right_mn_;
    ros::Publisher mecanum_back_rightPublisher_;
    std_msgs::Float64 mecanum_back_right_cmd_;
    double mecanum_back_right_fbk_;






};  // class

}  // namespace

#endif
