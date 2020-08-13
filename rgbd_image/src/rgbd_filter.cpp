#include "rgbd_class.h"
using namespace cv;
int main(int argc, char ** argv)
{
  ros::init(argc,argv,"rgbd_filter");
  ros::NodeHandle node_handle("~");
  RGBDImage RGBDImage_filter;
  std::string single_transfer,show_flag;
  node_handle.param<std::string>("single_transfer_",single_transfer, "False");
  node_handle.param<std::string>("show_flag",show_flag, "False");
  if(RGBDImage_filter.input_type_option=="image")
  {
    for(int i=0;i<RGBDImage_filter.trans_count;i++){
      if(!ros::ok()) break;
      RGBDImage_filter.Get_Input_Image();
      ROS_INFO("No.%u.png",RGBDImage_filter.current_num);
      RGBDImage_filter.Filter_Image();
      // RGBDImage_filter.Write_Output();
      if(single_transfer=="True") break;
    }
  }
  if(single_transfer=="True"&&show_flag=="True")
  {
    std::cout<<"Show Image!"<<std::endl;
    cv::Mat input_img,show_img;
    show_img = RGBDImage_filter.Output_Depth_Image();
    input_img = RGBDImage_filter.Input_Depth_Image();
    while(waitKey(0)!=27)
    {
      imshow("original", show_img);	
      imshow("filter", show_img);	
    }
  }
  return 0;
}