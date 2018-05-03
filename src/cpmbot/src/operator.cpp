#include "ros/ros.h" 
#include "std_msgs/String.h" 
#include "sensor_msgs/Joy.h"
#include <iostream>  
#include <cpmbot/cpmbot_basic.h>

class OperatorCommands
{
public:
OperatorCommands(ros::NodeHandle n ) 
{
  // joystick parameters
  n.param("joy/button/mecanum_movement_mode", button_mecanum_movement_mode, -1);
  n.param("joy/button/arm_position_mode", button_arm_position_mode_, -1);
  n.param("joy/button/arm_angle_reset", button_arm_angle_reset_, -1);
  n.param("joy/button/arm_front_up", button_arm_front_up_, -1);
  n.param("joy/button/arm_front_down", button_arm_front_down_, -1);
  n.param("joy/button/arm_back_up", button_arm_back_up_, -1);
  n.param("joy/button/arm_back_down", button_arm_back_down_, -1);

  n.param("joy/axe/x", axe_x_, 1);
  n.param("joy/axe/y", axe_y_, 0);
  n.param("joy/axe/zrot", axe_zrot_, 2);
  n.param("joy/axe/arm_front_left", axe_arm_front_left_, -1);
  n.param("joy/axe/arm_front_right", axe_arm_front_right_, -1);
  n.param("joy/axe/arm_back_left", axe_arm_back_left_, -1);
  n.param("joy/axe/arm_back_right", axe_arm_back_right_, -1);

  //operator parameters
  n.param("operator/arm_activate_buttons", button_arm_activate_, false);
  n.param("operator/mult/x", mult_x_, 1.0);
  n.param("operator/mult/y", mult_y_, 1.0);
  n.param("operator/mult/zrot", mult_zrot_, 1.0);
  n.param("operator/mult/arm_front_left", mult_arm_front_left_, 1.0);
  n.param("operator/mult/arm_front_right", mult_arm_front_right_, 1.0);
  n.param("operator/mult/arm_back_left", mult_arm_back_left_, 1.0);
  n.param("operator/mult/arm_back_right", mult_arm_back_right_, 1.0);
  n.param("operator/offset/arm_front_left", offset_arm_front_left_, 0.0);
  n.param("operator/offset/arm_front_right", offset_arm_front_right_, 0.0);
  n.param("operator/offset/arm_back_left", offset_arm_back_left_, 0.0);
  n.param("operator/offset/arm_back_right", offset_arm_back_right_, 0.0);

  n.param<std::string>("topic/subscriber/joystick", joy_topic_, "joy");
  n.param<std::string>("topic/publisher/basic_command", basic_command_topic_, "basic/command");

  JoystickSubscriber_ =  n.subscribe(joy_topic_,10, &OperatorCommands::JoyCallback ,this);
  BasicCmdPublisher_ = n.advertise<cpmbot::cpmbot_basic>(basic_command_topic_, 10);

}

void load() 
{
  //set velocity mode true by default
  cmdBasicMovement_.arm_velocity_mode = true;
}

private:
  std::string joy_topic_;
  std::string basic_command_topic_;

	ros::Publisher BasicCmdPublisher_;
	ros::Subscriber JoystickSubscriber_;

	cpmbot::cpmbot_basic cmdBasicMovement_;

  int button_arm_angle_reset_;
	int button_mecanum_movement_mode;
  int button_arm_position_mode_;

  //parameter xyz axis
  int axe_x_;
  int axe_y_;
  int axe_zrot_;

  double mult_x_;
  double mult_y_;
  double mult_zrot_;

  //parameter arms
  int axe_arm_front_left_;
  int axe_arm_front_right_;
  int axe_arm_back_left_;
  int axe_arm_back_right_;

  bool button_arm_activate_;
  int button_arm_front_up_;
  int button_arm_front_down_;
  int button_arm_back_up_;
  int button_arm_back_down_;

  double mult_arm_front_left_;
  double mult_arm_front_right_;
  double mult_arm_back_left_;
  double mult_arm_back_right_;
  double offset_arm_front_left_;
  double offset_arm_front_right_;
  double offset_arm_back_left_;
  double offset_arm_back_right_;


void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg) 
  { 
    double axes[] = {msg->axes[0],
  		msg->axes[1],
  		msg->axes[2],
   		msg->axes[3],
   		msg->axes[4],
   		msg->axes[5],
   		msg->axes[6],
   		msg->axes[7]};

 		int buttons[] = {msg->buttons[0],
			msg->buttons[1],
			msg->buttons[2],
			msg->buttons[3],
			msg->buttons[4],
			msg->buttons[5],
			msg->buttons[6],
			msg->buttons[7],
			msg->buttons[8],
			msg->buttons[9],
			msg->buttons[10],
			msg->buttons[11],
			msg->buttons[12],
			msg->buttons[13],
			msg->buttons[14]};


    cmdBasicMovement_.type_sender = 1;
    cmdBasicMovement_.type_command_filter = 1;

    if(button_mecanum_movement_mode >= 0){
      cmdBasicMovement_.mecanum_movement_mode = buttons[button_mecanum_movement_mode];
    }
    if(button_arm_position_mode_ >= 0){
      cmdBasicMovement_.arm_velocity_mode = not(buttons[button_arm_position_mode_]);
    }
    if(button_arm_angle_reset_ >= 0){
      cmdBasicMovement_.arm_angle_reset = buttons[button_arm_angle_reset_];
    }
      
    cmdBasicMovement_.robot_x = mult_x_ * axes[axe_x_];
    cmdBasicMovement_.robot_y = mult_y_ * axes[axe_y_];
    cmdBasicMovement_.robot_zrot = mult_zrot_ * axes[axe_zrot_];
      
    if((axe_arm_front_left_ >= 0)&&(axe_arm_front_right_ >= 0)&&(axe_arm_back_left_ >= 0)&&(axe_arm_back_right_ >= 0)){
      cmdBasicMovement_.arm_front_left = mult_arm_front_left_ * (axes[axe_arm_front_left_]+ offset_arm_front_left_);
      cmdBasicMovement_.arm_front_right = mult_arm_front_right_ * (axes[axe_arm_front_right_]+ offset_arm_front_right_);
      cmdBasicMovement_.arm_back_left = mult_arm_back_left_ * (axes[axe_arm_back_left_]+ offset_arm_back_left_);
      cmdBasicMovement_.arm_back_right = mult_arm_back_right_ * (axes[axe_arm_back_right_]+ offset_arm_back_right_);}
      
    if((button_arm_front_up_ >= 0)&&(button_arm_back_up_ >= 0)){
      //if joystick has no axis for the arms, then give full up or down command
      if(button_arm_activate_ == true){
        cmdBasicMovement_.arm_front_left = mult_arm_front_left_ * (buttons[button_arm_front_up_] - buttons[button_arm_front_down_]+ offset_arm_front_left_);
        cmdBasicMovement_.arm_front_right = mult_arm_front_right_ * (buttons[button_arm_front_up_] - buttons[button_arm_front_down_]+ offset_arm_front_right_);
        cmdBasicMovement_.arm_back_left = mult_arm_back_left_ * (buttons[button_arm_back_up_] - buttons[button_arm_back_down_]+ offset_arm_back_left_);
        cmdBasicMovement_.arm_back_right = mult_arm_back_right_ * (buttons[button_arm_back_up_] - buttons[button_arm_back_down_]+ offset_arm_back_right_);
      }
    }

    BasicCmdPublisher_.publish(cmdBasicMovement_);
  }
};

int main(int argc, char** argv) 
{ 
  ros::init(argc, argv,"operator_node"); 
 	ROS_INFO("in main");

	ros::NodeHandle n;

	OperatorCommands OpCommand(n);
  OpCommand.load();

  ros::spin();  	
} 