cmake_minimum_required(VERSION 2.8.3)
project(kinect)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs roscpp
)
find_package( OpenCV REQUIRED )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
 add_library(${PROJECT_NAME} SHARED
   src/kinect_pub.cpp
 )

AUX_SOURCE_DIRECTORY(src DIR_SRCS)
SET(Kinect_PATH ${DIR_SRCS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(kinect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/kinect_node.cpp src/kinect_pub.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_socket src/kinect_socket.cpp)
add_dependencies(${PROJECT_NAME}_socket ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME}_socket ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_socket ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test src/test.cpp)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME}_test ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_yolo src/kinect_yolo.cpp)
add_dependencies(${PROJECT_NAME}_yolo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME}_yolo ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_yolo ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_follow src/kinect_follow.cpp src/kinect_pub.cpp)
add_dependencies(${PROJECT_NAME}_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_follow ${catkin_LIBRARIES})
