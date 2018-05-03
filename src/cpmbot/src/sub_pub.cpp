    #include "ros/ros.h" 
    #include "std_msgs/String.h" 
    #include "std_msgs/Float64.h"

   class SubPub
   {
   public:

	SubPub(ros::NodeHandle n, std::string sub_topic_, std::string pub_topic_) 
	{


    


    Subscriber_ =  n.subscribe(sub_topic_,10, &SubPub::SubCallback ,this);
    Publisher_ = n.advertise<std_msgs::Float64>(pub_topic_, 10);

	}

  void load() 
  {

  }



private:

  std::string sub_topic_;
  std::string pub_topic_;

	ros::Publisher Publisher_;
	ros::Subscriber Subscriber_;


	void SubCallback(const std_msgs::Float64::ConstPtr& msg) 
    { 
    	
      
      Publisher_.publish(msg);
	
    }
	
};


     
 
    int main(int argc, char** argv) 
    { 
    ros::init(argc, argv,"SubPub_node"); 

 		ROS_INFO("in main");

    ros::NodeHandle n;

    std::string sub_front_left;
    std::string pub_front_left;
    std::string sub_front_right;
    std::string pub_front_right;
    std::string sub_back_left;
    std::string pub_back_left;
    std::string sub_back_right;
    std::string pub_back_right;

    n.param<std::string>("sub_front_left", sub_front_left, "sub_front_left");
    n.param<std::string>("pub_front_left", pub_front_left, "pub_front_left");
    n.param<std::string>("sub_front_right", sub_front_right, "sub_front_right");
    n.param<std::string>("pub_front_right", pub_front_right, "pub_front_right");
		n.param<std::string>("sub_back_left", sub_back_left, "sub_back_left");
    n.param<std::string>("pub_back_left", pub_back_left, "pub_back_left");
    n.param<std::string>("sub_back_right", sub_back_right, "sub_back_right");
    n.param<std::string>("pub_back_right", pub_back_right, "pub_back_right");

    SubPub front_left(n,sub_front_left, pub_front_left);
    SubPub front_right(n,sub_front_right, pub_front_right);
    //SubPub back_left(n,sub_back_left, pub_back_left);
    //SubPub back_right(n,sub_back_right, pub_back_right);



    ros::spin();
      	
    } 
