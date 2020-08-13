#ifndef GRID_MAP_TRANSFORM_FILTER_NODE_H_
#define GRID_MAP_TRANSFORM_FILTER_NODE_H_

#include <ros/ros.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

class TransformFilterNode {
 public:
  TransformFilterNode();

  // transform and filter cloud
  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg);

 private:
  ros::NodeHandle nh_;

  // subscriber
  ros::Subscriber sub_cloud_;

  // publisher
  ros::Publisher pub_cloud_;

  // transform matrix
  Eigen::Affine3f transform_matrix_;

  // topic name
  std::string in_cloud_topic_;
  std::string out_cloud_topic_;
  std::string frame_id_;

  // params
  double trans_roll_;
  double trans_pitch_;
  double trans_yaw_;
  double trans_tx_;
  double trans_ty_;
  double trans_tz_;
  double filter_voxel_size_;
  double filter_min_x_;
  double filter_max_x_;
  double filter_min_y_;
  double filter_max_y_;
  double filter_min_z_;
  double filter_max_z_;

  // remove radius outlier param
  bool is_use_radius_outlier_;
  double outlier_radius_;
  int outlier_min_neighbors_;

  // cloud pass filter
  void CloudPassFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);

  // cloud voxel filter
  void CloudVoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                        float in_voxel_size);

  // radius outlier removal filter
  void RadiusOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                        float radius, int min_neighbors);

  // remove points on self car
  void RemoveCarPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
};

#endif  // GRID_MAP_TRANSFORM_FILTER_NODE_H_
