#include "rgbd_class.h"

using namespace cv;

RGBDImage::RGBDImage()
{
    ros::NodeHandle pn("~");
    pn.param<std::string>("input_depth_dir_", input_depth_dir, "input_depth_dir");
    pn.param<std::string>("out_depth_dir_", out_depth_dir, "out_depth_dir");
    pn.param<std::string>("input_color_dir_", input_color_dir, "input_color_dir");
    pn.param<std::string>("out_color_dir_", out_color_dir, "out_color_dir");
    pn.param<std::string>("out_cloud_dir_", out_cloud_dir, "out_cloud_dir");
    pn.param<std::string>("input_type_option_", input_type_option, "input_type_option");
    pn.param<std::string>("output_type_option_", output_type_option, "output_type_option");
    pn.param<int>("trans_count_", trans_count, 10);
    pn.param<int>("start_num_", start_num, 1);
    pn.param<std::string>("single_transfer_", single_transfer, "False");
    pn.param<std::string>("single_depth_filename_", single_depth_filename, "filename");
    pn.param<std::string>("single_color_filename_", single_color_filename, "filename");
    pn.param<double>("camera_cx", camera_cx, 487.6446798734261);
    pn.param<double>("camera_cy", camera_cy, 279.5953266225303);
    pn.param<double>("camera_fx", camera_fx, 535.9643029929152);
    pn.param<double>("camera_fy", camera_fy, 537.1964560834192);
    //----------------------------------------------------------//
    pn.param<bool>("filter_bilateral_", filter_bilateral, false);  //双边滤波
    pn.param<int>("bilateralFilter_d", bilateralFilter_d, 0);
    pn.param<double>("bilateralFilter_sigmaColor", bilateralFilter_sigmaColor, 0);
    pn.param<double>("bilateralFilter_sigmaSpace", bilateralFilter_sigmaSpace, 0);
    //----------------------------------------------------------//

    current_num=start_num;
    cloud_xyz.points.clear();
}

void RGBDImage::Filter_Image()
{
    // depth_out_img = depth_img;
    // depth_out_img = color_img;
    std::cout<<"filter_bilateral: "<<filter_bilateral<<std::endl;
    std::cout<<bilateralFilter_sigmaColor<<std::endl;
    if(filter_bilateral)
        cv::bilateralFilter(color_img,depth_out_img,
                            bilateralFilter_d,
                            bilateralFilter_sigmaColor,
                            bilateralFilter_sigmaSpace);
}

cv::Mat RGBDImage::Output_Depth_Image()
{
    // cv::Mat temp_img;
    // temp_img = depth_out_img;
    if(depth_out_img.empty())
        std::cerr<<"Output Image ERROR!!"<<std::endl;
    return depth_out_img;
}

cv::Mat RGBDImage::Input_Depth_Image()
{
    // cv::Mat temp_img;
    // temp_img = depth_out_img;
    if(depth_img.empty())
        std::cerr<<"Input Image ERROR!!"<<std::endl;
    return color_img;
}

void RGBDImage::Get_Input_Image()
{
    Get_Input_Path();
    if(input_type_option=="image"){
        depth_img = cv::imread(input_depth_filename,-1);
        color_img = cv::imread(input_color_filename,-1);
        if (depth_img.empty()||color_img.empty())
            {std::cerr << "读取图片错误，请确认目录下是否有imread函数指定图片存在！"<<std::endl;exit(1);}
        else
            std::cout<<"Get Image!!!"<<std::endl;
    }
    else
        std::cerr<<"Wrong input type!!!"<<std::endl;
}

void RGBDImage::Get_Input_Path(){
    temp<<current_num;
    input_depth_filename="";
    input_color_filename="";
    if(input_type_option=="image")
    {
        input_depth_filename=input_depth_dir + "/" + temp.str() + ".png";
        input_color_filename=input_color_dir + "/" + temp.str() + ".png";
    }
    if(single_transfer=="True")
    {
        input_depth_filename = single_depth_filename;
        input_color_filename = single_color_filename;
    }
    temp.str("");
}

void RGBDImage::Write_Output()
{
    Get_Output_Path();
    if(output_type_option=="pcd"){ //bin2pcd  pcd2pcd 
      // std::cout<<output_filename<<std::endl;
        Write_pcd(depth_out_img);
    }
    else if(output_type_option=="image"){ //pcd2bin  bin2bin 
      // std::cout<<output_filename<<std::endl;
        Write_Image();
    }
    Update_path();
}


void RGBDImage::Get_Output_Path(){
    temp<<current_num;
    output_depth_filename="";
    output_color_filename="";
    output_cloud_filename="";
    if(output_type_option=="pcd")
        output_cloud_filename=out_cloud_dir + "/" + temp.str() + ".pcd";
    else if(output_type_option=="image")
    {
        output_depth_filename=out_depth_dir + "/" + temp.str() + ".png";
        output_color_filename=out_color_dir + "/" + temp.str() + ".png";     
    }
    else if(output_type_option=="count")
        ;
    temp.str("");
}

void RGBDImage::Update_path()
{
    temp.str("");
    current_num++;
}

void RGBDImage::Write_pcd(cv::Mat depth_img)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_depth (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    pcl::PointXYZ P;
    float d;
    cloud_xyz.points.clear();
    if(depth_img.empty())
        {std::cerr << "读取图片错误，请确认目录下是否有imread函数指定图片存在！"<<std::endl;exit(1);}
    else{
        for(int r = 0; r < depth_img.rows; ++r)
        {
            const uint16_t *itI = depth_img.ptr<uint16_t>(r);
            for(int c = 0; c < depth_img.cols; ++c, ++itI)
            {
                d = *itI/1000.0f;
                if(d!=0)
                {
                    P.z=d;
                    P.x=(c-camera_cx)*P.z/camera_fx;
                    P.y=(r-camera_cy)*P.z/camera_fy;
                    cloud_depth->points.push_back(P);
                }
            }
        }
        cloud_depth->height=1;
        cloud_depth->width=cloud_depth->points.size();
        cloud_depth->is_dense=false;
        cloud_xyz = *cloud_depth;
        if(cloud_depth->points.size()!=0)
        {
            voxelgrid.setInputCloud(cloud_depth);//输入点云数据
            voxelgrid.setLeafSize(0.03f, 0.03f, 0.03f);//AABB长宽高,设置体素栅格在XYZ3个方向上的尺寸
            voxelgrid.filter(cloud_xyz);
        }
        pcl::io::savePCDFile(output_cloud_filename, cloud_xyz);
    }
}

void RGBDImage::Write_Image()
{
    if(depth_out_img.empty())
        {std::cerr << "输出图片错误，请确认输出图片是否存在数据！"<<std::endl;exit(1);}
    else
    {
        cv::imwrite(output_depth_filename,depth_out_img);
        std::cout<<"Write Image: "<<output_depth_filename<<std::endl;
    }
}