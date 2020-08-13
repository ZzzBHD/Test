#ifndef PALLET_TOP
#define PALLET_TOP

#include "ros/ros.h"
#include "pallet_detection/pcl_head.h"
#include "pallet_detection/pcl_filter.h"
#include "pallet_detection/opencv_head.h"
#include "pallet_detection/plane_fitting_ground_segmenter.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "grid_map/map_generation.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/region_growing_rgb.h> 
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"  
#include <vector>
#include <cmath>
#include <math.h>
#include <omp.h>
#include <time.h>

using namespace cv;

class Pallet
{
public:
    Pallet(ros::NodeHandle nh);
    Pallet();
    ~Pallet(){};
    void Pallet_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud);
    void Init();
    void Pallet_Extract_collect(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PassThrough<pcl::PointXYZ> passthrough,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,
        int normal_flag);
    void Pallet_Extract_match(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PassThrough<pcl::PointXYZ> passthrough,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);
    void Pallet_Match(std::vector<cv::Mat> object_grid,bool morphology,int model);
    void Pallet_Grid(std::vector<pcl::PointCloud<pcl::PointXYZ>> object_cluster);
    void Pallet_Collect(std::vector<cv::Mat> object_grid,bool morphology);
    void Read_ImgModel(std::string load_path);
    void Read_FeatureModel();
    int frame;
    static const int Feature_num = 4;
    cv::Mat input_img;
    std::string load_img_path;
    std::string load_feature_path;
    std::string load_canny_path;
    std::vector<std::vector<double>> object_feature_model;
    std::vector<cv::Mat> object_img_model;
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr offland_cloud;

private:
    MapGeneration GridMap;
    bool Hu_tran_flag;
    bool error_flag;
    bool get_flag;
    int show_flag;
    int pass_z_min;
    int pass_z_max;
    int region_mincluster;
    int region_maxcluster;
    int pca_ksearch;
    int pca_rad;
    double region_angle;
    double region_curve;
    double range_min_x_;
    double range_min_y_;
    double range_max_x_;
    double range_max_y_;
    int model_number;
    int match_number;
    int pixel_scale_;
    int cut_h;
    int cut_w;
    int map_height_;
    int map_width_;
    int map_height_origin_;
    int map_width_origin_;
    double Feature_Threshold;
    double Img_Threshold;

    std::string model;
    std::string save_img_path;
    std::string save_feature_path;

    ros::Publisher pallet_get_publisher;
    std::string pallet_get_topic;
    std::string tof_topic;
    ros::Subscriber tof_subscriber;
    // ros::Publisher pose_publisher;
    ros::Publisher target_publisher;

    pcl::PassThrough<pcl::PointXYZ> passthrough;
    cv_bridge::CvImagePtr cv_ptr;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    double Feature[Feature_num];
    std::vector<int> colour;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> object_cluster;
    std::vector<pcl::PointCloud<pcl::Normal>> object_normal;
    std::vector<pcl::PointXYZ> object_avgnormal;
    std::vector<cv::Mat> object_grid;
    std::vector<double> Hu;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<double> CalHu(cv::Mat dst);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pipline(PointTypeCloudPtr original_cloud_ptr);
    std::vector<pcl::PointIndices> RegionGrow_extract(
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        int show_flag);
    pcl::PointCloud<pcl::Normal> NormalEstimationPCA(
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<pcl::PointXYZ> CalNormals(
        std::vector<pcl::PointCloud<pcl::Normal>> object_normal);
    void Show_Normals(
        std::vector<pcl::PointCloud<pcl::PointXYZ>> object_cluster,
        std::vector<pcl::PointCloud<pcl::Normal>> object_normal);
    void FillHole(const Mat srcBw, Mat &dstBw);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Filter_cloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PassThrough<pcl::PointXYZ> passthrough);
    double Pallet_judge(double Feature[4]);
    double Pallet_judge(cv::Mat object_img);
    const std::vector<int> Cluster_Colour(int n);
    void Tof_callback(const sensor_msgs::ImageConstPtr& msg);
};

#endif