cmake_minimum_required(VERSION 2.8.3)
# project(rosbag_to_pcd)
project(rgbd_image)
set(OpenCV_DIR "/home/cyber-z/opencv-3.4.6/build") 
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge 
  image_transport
  pcl_conversions 
  rospy
)
find_package( OpenCV REQUIRED )
find_package( PCL 1.3 REQUIRED )
find_package(Boost REQUIRED COMPONENTS
thread)
## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}${OpenCV_INCLUDE_DIRS})
include_directories(
  include ${PCL_INCLUDE_DIRS})
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
  src)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable

add_executable(rgbd_filter
  src/rgbd_filter.cpp
  src/rgbd_class.cpp
)
add_dependencies(rgbd_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rgbd_filter
  ${catkin_LIBRARIES}
)
target_link_libraries (rgbd_filter ${OpenCV_LIBS})