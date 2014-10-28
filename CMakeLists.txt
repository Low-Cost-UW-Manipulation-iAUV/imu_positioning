cmake_minimum_required(VERSION 2.8.3)
project(imu_positioning)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	std_msgs
	std_srvs
	sensor_msgs
	message_generation
	)
 add_message_files(
 	DIRECTORY msg
	FILES xsens_MTi.msg)

generate_messages(
	DEPENDENCIES
	std_msgs
	std_srvs
	sensor_msgs
	geometry_msgs
	)

## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS 
	message_runtime
	roscpp
	std_msgs
	std_srvs
	sensor_msgs
	geometry_msgs

	INCLUDE_DIRS include
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} SYSTEM )
set(SENSOR_INTEGRATION_SRCS src/sensor_integration.cpp src/trapezoid_integration.cpp src/calibration.cpp)
add_executable(xsens src/xsens_MTi30.cpp  ${SENSOR_INTEGRATION_SRCS})
target_link_libraries(xsens ${catkin_LIBRARIES}  ${roscpp_LIBRARIES} )
add_dependencies(xsens std_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
