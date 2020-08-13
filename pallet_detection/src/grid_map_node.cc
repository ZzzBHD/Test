#include "grid_map/grid_map_node.h"

using namespace cv;

GridMapNode::GridMapNode() : nh_("~") {
  nh_.param<std::string>("obstacle_cloud_topic", obstacle_cloud_topic_,
                         "sync_cloud");
  nh_.param<std::string>("grid_map_topic", grid_map_topic_, "grid_map");
  nh_.param<std::string>("frame_id", frame_id_, "lidar_front");
  nh_.param<double>("range_min_x", range_min_x_, -100.0);
  nh_.param<double>("range_max_x", range_max_x_, 100.0);
  nh_.param<double>("range_min_y", range_min_y_, -60.0);
  nh_.param<double>("range_max_y", range_max_y_, 60.0);
  nh_.param<int>("pixel_scale", pixel_scale_, 5);

  map_height_ = static_cast<int>(range_max_x_ - range_min_x_) * pixel_scale_;
  map_width_ = static_cast<int>(range_max_y_ - range_min_y_) * pixel_scale_;
  map_height_origin_ = static_cast<int>(range_max_x_) * pixel_scale_;
  map_width_origin_ = static_cast<int>(range_max_y_) * pixel_scale_;

  map_generation_ = new MapGeneration();
  map_generation_->Init(map_height_origin_, map_width_origin_, pixel_scale_);

  sub_obstacle_cloud_ = nh_.subscribe(
      obstacle_cloud_topic_, 2, &GridMapNode::ObstacleCloudCallback, this);

  image_transport::ImageTransport it(nh_);
  pub_grid_map_ = it.advertise(grid_map_topic_, 2);
}

void GridMapNode::ObstacleCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_in) {
  // tranform ros msg to pcl msg
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_in, *obstacle_cloud);

  // generate grid map using obstacle cloud
  Mat grid_map(map_height_, map_width_, CV_8UC1, Scalar::all(0));
  map_generation_->Generate(obstacle_cloud, 255, &grid_map);

  sensor_msgs::ImagePtr image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", grid_map).toImageMsg();
  image_msg->header = cloud_in->header;
  image_msg->header.frame_id = frame_id_;
  pub_grid_map_.publish(image_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map_node");
  GridMapNode grid_map_node;
  ros::spin();

  return 0;
}
