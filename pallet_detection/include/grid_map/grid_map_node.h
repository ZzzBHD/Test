#ifndef CYBERTIGGO_PERCEPTION_GRID_MAP_GRID_MAP_NODE_H_
#define CYBERTIGGO_PERCEPTION_GRID_MAP_GRID_MAP_NODE_H_

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "map_generation.h"

class GridMapNode {
 public:
  GridMapNode();

  // project obstacle cloud to grid map
  void ObstacleCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

 private:
  ros::NodeHandle nh_;

  // subscriber
  ros::Subscriber sub_obstacle_cloud_;

  // publisher
  image_transport::Publisher pub_grid_map_;

  // topic name
  std::string obstacle_cloud_topic_;
  std::string grid_map_topic_;

  // instantiates of MapGeneration object
  MapGeneration* map_generation_;

  // grid map params
  std::string frame_id_;
  double range_min_x_;
  double range_max_x_;
  double range_min_y_;
  double range_max_y_;
  int map_height_;
  int map_width_;
  int map_height_origin_;
  int map_width_origin_;
  int pixel_scale_;
};

#endif  // CYBERTIGGO_PERCEPTION_GRID_MAP_GRID_MAP_NODE_H_
