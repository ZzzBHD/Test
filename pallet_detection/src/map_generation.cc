#include "grid_map/map_generation.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace cv;

MapGeneration::MapGeneration() {}

void MapGeneration::Init(int height_origin, int width_origin, int pixel_scale) {
  map_height_origin_ = height_origin;
  map_width_origin_ = width_origin;
  pixel_scale_ = pixel_scale;
}

void MapGeneration::Generate(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_ptr,const uchar& scalar, cv::Mat* grid_map) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for(int i = 0; i < cloud_ptr->size(); ++i){
    pcl::PointXYZ pt = cloud_ptr->points[i];
    // if(pt.z<-0.7 && pt.z>-1.3)
    new_cloud->push_back(pt);
  }

  // std::cout<<"Map_cloud: "<<new_cloud->points.size()<<std::endl;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ>::Ptr rad(new pcl::RadiusOutlierRemoval<pcl::PointXYZ>());
  rad->setRadiusSearch(10);
  rad->setMinNeighborsInRadius(20);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  rad->setInputCloud(new_cloud);
  rad->filter(*filtered);
  filtered->header = cloud_ptr->header;
  // std::cout<<"Filter cloud: "<<filtered->size()<<std::endl;
  for (int i = 0; i < filtered->size(); ++i) {
    int colum, row;
    pcl::PointXYZ pt = filtered->points[i];
    RealToPixel(pt.x, pt.y, &row, &colum);
    if (row >= 0 && row < grid_map->rows && colum >= 0 &&
        colum < grid_map->cols)
      grid_map->at<uchar>(row, colum) = scalar;
  }
}

void MapGeneration::GenerateHeightMap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_ptr,
    const uchar& scalar, cv::Mat* grid_map, cv::Mat* max_height_map, cv::Mat* min_height_map) {
  for (int i = 0; i < cloud_ptr->size(); ++i) {
    int colum, row;
    pcl::PointXYZ pt = cloud_ptr->points[i];
    RealToPixel(pt.x, pt.y, &row, &colum);
    if (row >= 0 && row < grid_map->rows && colum >= 0 &&
        colum < grid_map->cols)
      grid_map->at<uchar>(row, colum) = scalar;
    float max_height = max_height_map->at<float>(row, colum);
    if (max_height < pt.z) {
      max_height_map->at<float>(row, colum) = pt.z;
    }
    float min_height = min_height_map->at<float>(row, colum);
    if (min_height > pt.z) {
      min_height_map->at<float>(row, colum) = pt.z;
    }
  }
}

void MapGeneration::RealToPixel(const float& x, const float& y, int* px,
                                int* py) {
  *px = map_height_origin_ - static_cast<int>(x * pixel_scale_);
  *py = map_width_origin_ - static_cast<int>(y * pixel_scale_);
}

void MapGeneration::PixelToReal(const int& px, const int& py, float* x,
                                float* y) {
  *x = static_cast<float>(map_height_origin_ - px) / pixel_scale_;
  *y = static_cast<float>(map_width_origin_ - py) / pixel_scale_;
}
