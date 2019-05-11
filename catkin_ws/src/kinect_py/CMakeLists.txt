cmake_minimum_required(VERSION 2.8.3)
project(kinect_py)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs rospy
)
find_package( OpenCV REQUIRED )


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   MsgKinect.msg
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )


catkin_package(
  LIBRARIES kinect
  CATKIN_DEPENDS rospy std_msgs
)


## Declare a C++ library
#add_library(${PROJECT_NAME} SHARED
#   src/kinect_pub.cpp
# )

#AUX_SOURCE_DIRECTORY(src DIR_SRCS)
#SET(Kinect_PATH ${DIR_SRCS})
