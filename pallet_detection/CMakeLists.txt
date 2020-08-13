cmake_minimum_required(VERSION 2.8.3)
project(pallet)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_definitions(-std=c++11)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
   message_generation std_msgs roscpp sensor_msgs cv_bridge image_transport pcl_ros pcl_conversions
   rospy
)

find_package( OpenCV REQUIRED )
find_package( PCL 1.3 REQUIRED )
find_package(Boost REQUIRED COMPONENTS
thread)
include_directories(
  include ${catkin_INCLUDE_DIRS}${OpenCV_INCLUDE_DIRS})
include_directories(
  include ${PCL_INCLUDE_DIRS})
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  LIBRARIES kinect
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport jsk_recognition_msgs
)


add_executable(pallet_node
  main.cpp
  pallet_top.cpp
  src/map_generation.cc
  src/plane_fitting_ground_segmenter.cpp
)
add_dependencies(pallet_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pallet_node
  ${catkin_LIBRARIES}
)
target_link_libraries (pallet_node ${OpenCV_LIBS})


