#ifndef GRID_MAP_RADIAL_SEARCH_NODE_H_
#define GRID_MAP_RADIAL_SEARCH_NODE_H_

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <opencv/cv.h>

class RadialSearchNode {
 public:
  RadialSearchNode();

  // remove ground points using radial search method
  void PointCloudCallback(
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_sensor_cloud);

 private:
  ros::NodeHandle nh_;

  // subscriber
  ros::Subscriber sub_cloud_;

  // publisher
  ros::Publisher pub_obstacle_;
  ros::Publisher pub_ground_;

  // topic name
  std::string point_cloud_topic_;
  std::string obstacle_cloud_topic_;
  std::string ground_cloud_topic_;

  // params
  std::string frame_id_;
  int vertical_res_;
  int horizontal_res_;
  double max_slope_;
  int min_points_;
  double point_radial_distance_;
  double point_height_distance_;
  double max_ratio_;
  double hang_radial_distance_;
  cv::Mat index_map_;

  // generate index map for radial search
  void GenerateIndexMap(
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_cloud);

  // publish cloud
  void PublishCloud(
      const ros::Publisher* in_publisher,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr);
};

#endif  // CYBERTIGGO_PERCEPTION_SENSOR_PREPROCESS_LIDAR_PREPROCESS_RADIAL_SEARCH_NODE_H_
