#ifndef CYBERTIGGO_PERCEPTION_GRID_MAP_MAP_GENERATION_H_
#define CYBERTIGGO_PERCEPTION_GRID_MAP_MAP_GENERATION_H_

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

class MapGeneration {
 public:
  MapGeneration();

  // initialize parameters
  void Init(int height_origin, int width_origin, int pixel_scale);

  // project pcl::PointCloud into cv::Mat
  void Generate(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_ptr,
                const uchar& scalar, cv::Mat* grid_map);

  // project pcl::PointCloud into cv::Mat with height
  void GenerateHeightMap(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_ptr,
      const uchar& scalar, cv::Mat* grid_map, cv::Mat* max_height_map,
      cv::Mat* min_height_map);

  // coordination transform: real to pixel
  void RealToPixel(const float& x, const float& y, int* px, int* py);

  // coordination transform: pixel to real
  void PixelToReal(const int& px, const int& py, float* x, float* y);

 private:
  // transform parames
  int map_height_origin_;
  int map_width_origin_;
  int pixel_scale_;
};

#endif  // CYBERTIGGO_PERCEPTION_GRID_MAP_MAP_GENERATION_H_
