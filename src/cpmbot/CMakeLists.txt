cmake_minimum_required(VERSION 2.8.3)
project(cpmbot)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_conversions
  	pcl_ros
	sensor_msgs
	std_msgs
	geometry_msgs
	message_generation

)

## Generate messages in the 'msg' folder
 add_message_files(FILES
   	cpmbot_basic.msg
   	cpmbot_detail.msg
 )
## Generate added messages and services with any dependencies listed here
 generate_messages(
	DEPENDENCIES
   	std_msgs 
 )






catkin_package(
#  INCLUDE_DIRS 
#	include
#  LIBRARIES 
  CATKIN_DEPENDS 
	roscpp 
	sensor_msgs 
	std_msgs
	geometry_msgs
   	message_runtime
   	pcl_conversions
  	pcl_ros
#  DEPENDS
)






include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)


add_executable(operator_node src/operator.cpp )#${catkin_LIBRARIES} )
add_dependencies(operator_node ${catkin_EXPORTED_TARGETS})

add_executable(basic_movement_node src/basic_movement.cpp ) #${catkin_LIBRARIES} )
add_dependencies(basic_movement_node ${catkin_EXPORTED_TARGETS})

add_executable(sub_pub_node src/sub_pub.cpp ) #${catkin_LIBRARIES} )
add_dependencies(sub_pub_node ${catkin_EXPORTED_TARGETS})

add_executable(arosa src/arosa.cpp)
add_dependencies(arosa ${catkin_EXPORTED_TARGETS})


target_link_libraries(operator_node ${catkin_LIBRARIES})
target_link_libraries(basic_movement_node ${catkin_LIBRARIES})
target_link_libraries(sub_pub_node ${catkin_LIBRARIES})
target_link_libraries(arosa ${catkin_LIBRARIES})
