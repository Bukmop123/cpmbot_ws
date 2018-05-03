    #include "ros/ros.h" 
    #include "std_msgs/Bool.h" 
    #include "std_msgs/String.h" 
    #include "std_msgs/Float64.h" 
    #include "geometry_msgs/Twist.h" 
    #include <unistd.h>
	//#include <cmath.h>

    #include "sensor_msgs/Imu.h"
    #include "sensor_msgs/JointState.h"


  	//#include "mybot_msg/msgMybot_jointInterface.h"
    #include <cpmbot/cpmbot_basic.h>
    #include <cpmbot/cpmbot_detail.h>


 

  double rosTimeToDouble(ros::Time RosTime) //ToDo implement in a library
  { 
    double a;
    double b;
    double c;
    a=RosTime.sec;
    b=RosTime.nsec;
    c=1.0*a + 1.0*b*10.0e-10;
    return c;
  } 

  class BasicMovementControl
  {

  public:

	  BasicMovementControl(ros::NodeHandle n){
   		ROS_INFO("in emty constructor");
      int type_sender = 0;
      int type_command_filter = 0;
      bool mecanum_movement_mode = false; 
      bool arm_velocity_mode = false; 
      bool arm_angle_reset = false;

      robot_x = 0; 
      robot_y = 0; 
      robot_zrot = 0; 
      arm_front_left_cmd_ = 0; 
      arm_front_right_cmd_ = 0; 
      arm_back_left_cmd_ = 0;  
      arm_back_right_cmd_ = 0; 

      

      n.param("parameter/hardware/wheel_radius", wheel_radius, 1.0);
      n.param("parameter/hardware/robot_center_to_wheel_width", robot_width, 1.0);
      n.param("parameter/hardware/robot_center_to_wheel_length", robot_length, 1.0);

      n.param<std::string>("topic/subscriber/basic_command", basic_command_topic_, "basic/command");

      n.param<std::string>("topic/publisher/base_link_command", base_link_controller_topic, "basic/command");
      n.param<std::string>("topic/publisher/arm/front_left_command", arm_front_left_controller_topic, "arm/front_left/command");
      n.param<std::string>("topic/publisher/arm/front_right_command", arm_front_right_controller_topic, "arm/front_right/command");
      n.param<std::string>("topic/publisher/arm/back_left_command", arm_back_left_controller_topic, "arm/back_left/command");
      n.param<std::string>("topic/publisher/arm/back_right_command", arm_back_right_controller_topic, "arm/back_right/command");
      n.param<std::string>("topic/publisher/mecanum/front_left_command", mecanum_front_left_controller_topic, "mecanum/front_left/command");
      n.param<std::string>("topic/publisher/mecanum/front_right_command", mecanum_front_right_controller_topic, "mecanum/front_right/command");
      n.param<std::string>("topic/publisher/mecanum/back_left_command", mecanum_back_left_controller_topic, "mecanum/back_left/command");
      n.param<std::string>("topic/publisher/mecanum/back_right_command", mecanum_back_right_controller_topic, "mecanum/back_right/command");

      n.param<std::string>("topic/publisher/detail_command", detail_command_topic_, "detail/command");


      BasicCmdSubscriber_ =  n.subscribe(basic_command_topic_,10, &BasicMovementControl::BasicCmdCallback ,this); 

      BaseLinkVelPublisher_ = n.advertise<geometry_msgs::Twist>(base_link_controller_topic, 10);
      ArmFrontLeftPublisher_ = n.advertise<std_msgs::Float64>(arm_front_left_controller_topic, 10);
      ArmFrontRightPublisher_ = n.advertise<std_msgs::Float64>(arm_front_right_controller_topic, 10);
      ArmBackLeftPublisher_ = n.advertise<std_msgs::Float64>(arm_back_left_controller_topic, 10);
      ArmBackRightPublisher_ = n.advertise<std_msgs::Float64>(arm_back_right_controller_topic, 10);
      MecanumFrontLeftPublisher_ = n.advertise<std_msgs::Float64>(mecanum_front_left_controller_topic, 10);
      MecanumFrontRightPublisher_ = n.advertise<std_msgs::Float64>(mecanum_front_right_controller_topic, 10);
      MecanumBackLeftPublisher_ = n.advertise<std_msgs::Float64>(mecanum_back_left_controller_topic, 10);
      MecanumBackRightPublisher_ = n.advertise<std_msgs::Float64>(mecanum_back_right_controller_topic, 10);

      DetailCommandsPublisher_ = n.advertise<cpmbot::cpmbot_detail>(detail_command_topic_, 10);

   	}

	  ~BasicMovementControl(){}

	  void Reset(){}

	  void Load(){

		ROS_INFO("in Load");
		//init 
    

	  }

    void loop()
    {
        ROS_INFO("in loop");
        baseLinkMovement();
        armAngleMovement();
        mecaumMovement();
        pubDetail();
                
    }



  private:
    

    std::string basic_command_topic_;
    std::string detail_command_topic_;
    std::string base_link_controller_topic;
    std::string arm_front_left_controller_topic;
    std::string arm_front_right_controller_topic;
    std::string arm_back_left_controller_topic;
    std::string arm_back_right_controller_topic;
    std::string mecanum_front_left_controller_topic;
    std::string mecanum_front_right_controller_topic;
    std::string mecanum_back_left_controller_topic;
    std::string mecanum_back_right_controller_topic;

    ros::Subscriber BasicCmdSubscriber_;

    ros::Publisher BaseLinkVelPublisher_ ;
    ros::Publisher ArmFrontLeftPublisher_ ;
    ros::Publisher ArmFrontRightPublisher_ ;
    ros::Publisher ArmBackLeftPublisher_ ;
    ros::Publisher ArmBackRightPublisher_ ;
    ros::Publisher MecanumFrontLeftPublisher_ ;
    ros::Publisher MecanumFrontRightPublisher_;
    ros::Publisher MecanumBackLeftPublisher_ ;
    ros::Publisher MecanumBackRightPublisher_ ;

    ros::Publisher DetailCommandsPublisher_ ;

    geometry_msgs::Twist baseLinkCmd;
    cpmbot::cpmbot_detail detailCmd_;

    int type_sender;
    int type_command_filter;
    bool mecanum_movement_mode; 
    bool arm_velocity_mode; 
    bool arm_angle_reset;
    double robot_x; 
    double robot_y; 
    double robot_zrot; 
    double arm_front_left_cmd_ ;
    double arm_front_right_cmd_ ;
    double arm_back_left_cmd_ ;
    double arm_back_right_cmd_ ;
    std_msgs::Float64 mecanum_front_left ;
    std_msgs::Float64 mecanum_front_right ;
    std_msgs::Float64 mecanum_back_left ;
    std_msgs::Float64 mecanum_back_right ;

    std_msgs::Float64 arm_front_left ;
    std_msgs::Float64 arm_front_right ;
    std_msgs::Float64 arm_back_left ;
    std_msgs::Float64 arm_back_right ;

    double wheel_radius;
    double robot_width;
    double robot_length;
    double robot_length_width;
  	
    

    //Subscriber / Callback Functions
    void BasicCmdCallback(const cpmbot::cpmbot_basic::ConstPtr& cmd_msg)
    {
      type_sender = cmd_msg->type_sender; 
      type_command_filter = cmd_msg->type_command_filter; 

      mecanum_movement_mode = cmd_msg->mecanum_movement_mode; 
      arm_velocity_mode = cmd_msg->arm_velocity_mode; 
      arm_angle_reset = cmd_msg->arm_angle_reset; 

      robot_x = cmd_msg->robot_x; 
      robot_y = cmd_msg->robot_y; 
      robot_zrot = cmd_msg->robot_zrot; 
      arm_front_left_cmd_ = cmd_msg->arm_front_left; 
      arm_front_right_cmd_ = cmd_msg->arm_front_right; 
      arm_back_left_cmd_ = cmd_msg->arm_back_left;  
      arm_back_right_cmd_ = cmd_msg->arm_back_right; 
    }

    //Process Functions

    //Publisher Functions
    void baseLinkMovement()
    {

      baseLinkCmd.linear.x = robot_x;
      baseLinkCmd.angular.z = robot_zrot;

      BaseLinkVelPublisher_.publish(baseLinkCmd);
    }

    void armAngleMovement()
    {
      if (arm_velocity_mode == false){

        arm_front_left.data = arm_front_left_cmd_; 
        arm_front_right.data = arm_front_right_cmd_; 
        arm_back_left.data = arm_back_left_cmd_;  
        arm_back_right.data = arm_back_right_cmd_; 

      }else if(arm_velocity_mode == true){
        arm_front_left.data = arm_front_left.data + arm_front_left_cmd_; 
        arm_front_right.data = arm_front_right.data + arm_front_right_cmd_; 
        arm_back_left.data = arm_back_left.data + arm_back_left_cmd_;  
        arm_back_right.data = arm_back_right.data + arm_back_right_cmd_; 

      }else if(arm_angle_reset == true){
        arm_front_left.data = 0; 
        arm_front_right.data = 0; 
        arm_back_left.data = 0;  
        arm_back_right.data = 0; 
      }


      ArmFrontLeftPublisher_.publish(arm_front_left);
      ArmFrontRightPublisher_.publish(arm_front_right);
      ArmBackLeftPublisher_.publish(arm_back_left);
      ArmBackRightPublisher_.publish(arm_back_right);
    }

    void mecaumMovement()
    {
      robot_length_width = robot_width + robot_length;

      mecanum_front_left.data = (robot_x - robot_y + robot_length_width * robot_zrot)/wheel_radius;
      mecanum_front_right.data = (robot_x + robot_y - robot_length_width * robot_zrot)/wheel_radius;
      mecanum_back_left.data = (robot_x + robot_y + robot_length_width * robot_zrot)/wheel_radius;
      mecanum_back_right.data = (robot_x - robot_y - robot_length_width * robot_zrot)/wheel_radius;

      MecanumFrontLeftPublisher_.publish(mecanum_front_left);
      MecanumFrontRightPublisher_.publish(mecanum_front_right);
      MecanumBackLeftPublisher_.publish(mecanum_back_left);
      MecanumBackRightPublisher_.publish(mecanum_back_right);
    }

    void pubDetail()
    {
      detailCmd_.robot_x = robot_x;
      detailCmd_.robot_y = robot_y;
      detailCmd_.robot_zrot = robot_zrot;
      detailCmd_.arm_front_left = arm_front_left.data;
      detailCmd_.arm_front_right = arm_front_right.data;
      detailCmd_.arm_back_left = arm_back_left.data;
      detailCmd_.arm_back_right = arm_back_right.data;
      detailCmd_.mecanum_front_left = mecanum_front_left.data;
      detailCmd_.mecanum_front_right = mecanum_front_right.data;
      detailCmd_.mecanum_back_left = mecanum_back_left.data;
      detailCmd_.mecanum_back_right = mecanum_back_right.data;
      DetailCommandsPublisher_.publish(detailCmd_);
    }



  };


     
 
    int main(int argc, char** argv) 
    {
        ros::init(argc, argv,"basic_movement_node"); 
        double cycle_time;
        double actualTime;
        double lastLoopTime;

        ROS_INFO("in main");
    
        ros::NodeHandle n;
        n.param("basic_movement/cycle_time", cycle_time, 0.1);
      
        BasicMovementControl BasicMovementController(n);
        BasicMovementController.Load();

        //ros::spin(); 

      
        while (ros::ok())
        {
        actualTime = rosTimeToDouble( ros::Time::now());

        if((actualTime-lastLoopTime) >= cycle_time){
          BasicMovementController.loop();
          lastLoopTime = actualTime;
        }
          
          ros::spinOnce();
          //usleep(100);
          //sleep(1);

        }

        ROS_INFO("in main end");

    } 