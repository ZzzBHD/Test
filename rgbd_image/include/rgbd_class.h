#pragma once

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include <string.h>
#include <boost/program_options.hpp>
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv_head.h"
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件

class RGBDImage
{
public:
  RGBDImage();
  ~RGBDImage(){}
  void Get_Input_Image();
  void Write_Output();
  void Filter_Image();
  cv::Mat Output_Depth_Image();
  cv::Mat Input_Depth_Image();
  // void Trans_Init();
  void Update_path();
  // sensor_msgs::PointCloud2 Point2Sensor();
  std::string input_type_option,output_type_option;
  int start_num,trans_count,current_num;
  // int count_point;
  // std::string cloud_type;


private:
  // ros::Subscriber pub_subscriber;
  // ros::Subscriber tof_subscriber;
  std::string input_depth_dir, input_color_dir, out_depth_dir, out_color_dir,out_cloud_dir;
  std::string input_depth_filename,input_color_filename,output_depth_filename, output_color_filename,output_cloud_filename;
  std::string single_transfer,single_depth_filename,single_color_filename;
  bool filter_bilateral;
  cv::Mat depth_img;
  cv::Mat depth_out_img;
  cv::Mat color_img;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  // pcl::PointCloud<pcl::PointXYZI> cloud_xyzi;
  std::stringstream temp;
  void Get_Input_Path();
  void Get_Output_Path();
  void Write_Image();
  void Write_pcd(cv::Mat depth_img);
  double camera_cx,camera_cy,camera_fx,camera_fy;

  int bilateralFilter_d;//在过滤过程中每个像素邻域的直径
  double bilateralFilter_sigmaColor; //颜色空间滤波器的sigma值
  //这个参数的值越大，就表明该像素邻域内有更宽广的颜色会被混合到一起，产生较大的半相等颜色区域。范围越大图像越模糊
  double bilateralFilter_sigmaSpace; //坐标空间中滤波器的sigma值
  //数值越大，意味着越远的像素会相互影响，从而使更大的区域足够相似的颜色获取相同的颜色。
  //当bilateralFilter_d>0，d指定了邻域大小且与sigmaSpace无关。否则，d正比于sigmaSpace。

};