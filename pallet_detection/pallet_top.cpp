#include "pallet_top.h"
#include <fstream>
#include "yaml-cpp/yaml.h"

using namespace cv;

GroundPlaneFittingSegmenterParams params;
GroundPlaneFittingSegmenter ground_segmenter(params);


Pallet::Pallet(ros::NodeHandle n)
  :Hu_tran_flag(0),
   error_flag(0),
   show_flag(2),
   get_flag(0),
   frame(1)
{
    ros::NodeHandle nh("~");
    std::string topic;
    int queue_size;
    nh.param("ros_default_queue_size", queue_size, 1);

    nh.param<bool>("Hu_tran_flag", Hu_tran_flag, 1);
    nh.param<int>("pass_z_min", pass_z_min, 4);
    nh.param<int>("pass_z_max", pass_z_max, 12);
    // std::cout<<pass_z_max<<std::endl;
    nh.param<double>("region_curve", region_curve, 10.0);
    nh.param<double>("region_angle", region_angle, 10.0);
    nh.param<int>("region_mincluster", region_mincluster, 100);
    nh.param<int>("region_maxcluster", region_maxcluster, 50000);
    nh.param<int>("pca_rad", pca_rad, 0);
    nh.param<int>("pca_ksearch", pca_ksearch, 10);
    nh.param<int>("model_number", model_number, 10);
    nh.param<std::string>("model", model, "mo-1");
    std::cout<<"Choose Model: "<<model<<std::endl;
    nh.param<double>("Feature_Threshold", Feature_Threshold, 0.5);
    nh.param<double>("Img_Threshold", Img_Threshold, 1.0);

    nh.param<std::string>("save_img_path", save_img_path, "/media/cyber-z/E/cache/img/");
    nh.param<std::string>("save_feature_path", save_feature_path, "/media/cyber-z/E/cache/feature/");
    nh.param<std::string>("load_img_path", load_img_path, "/media/cyber-z/E/cache/img/");
    nh.param<std::string>("load_feature_path", load_feature_path, "/media/cyber-z/E/cache/feature/1-0-feature.txt");
    nh.param<std::string>("load_canny_path", load_canny_path, "");

    nh.param<double>("range_min_x_", range_min_x_, -4.0);
    nh.param<double>("range_min_y_", range_min_y_, -4.0);
    nh.param<double>("range_max_x_", range_max_x_, 4.0);
    nh.param<double>("range_max_y_", range_max_y_, 4.0);
    nh.param<int>("pixel_scale_", pixel_scale_, 30);
    nh.param<int>("cut_h", cut_h, 0);
    nh.param<int>("cut_w", cut_w, 0);

    map_height_ = static_cast<int>(range_max_x_ - range_min_x_) * pixel_scale_ - cut_h;
    map_width_ = static_cast<int>(range_max_y_ - range_min_y_) * pixel_scale_ - cut_w;
    map_height_origin_ = static_cast<int>(range_max_x_) * pixel_scale_ - cut_h/2;
    map_width_origin_ = static_cast<int>(range_max_y_) * pixel_scale_ - cut_w/2;

    GridMap.Init(map_height_origin_, map_width_origin_, pixel_scale_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);original_cloud=cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud_(new pcl::PointCloud<pcl::PointXYZ>);filter_cloud=filter_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr offland_cloud_(new pcl::PointCloud<pcl::PointXYZ>);offland_cloud=offland_cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ (new pcl::search::KdTree<pcl::PointXYZ>);tree=tree_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_(new pcl::PointCloud<pcl::Normal>);cloud_normals=normals_;

    nh.param<std::string>("tof_topic", tof_topic, "/camera1/qhd/image_depth_rect");
    tof_subscriber = nh.subscribe(topic, queue_size, &Pallet::Tof_callback, this);

    nh.param<std::string>("pallet_get_topic", pallet_get_topic, "/pallet_get");
    pallet_get_publisher = nh.advertise<std_msgs::Bool>(pallet_get_topic, queue_size);
}

void Pallet::Init()
{
    object_cluster.clear();
    object_normal.clear();
    object_avgnormal.clear();
    object_grid.clear();
    object_feature_model.clear();
    object_img_model.clear();
    // Read_ImgModel(load_img_path);
    contours.clear();
}


void Pallet::FillHole(const Mat srcBw, Mat &dstBw)
{
    Size m_Size = srcBw.size();
    Mat Temp=Mat::zeros(m_Size.height+2,m_Size.width+2,srcBw.type());//延展图像
    srcBw.copyTo(Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)));
    cv::floodFill(Temp, Point(0, 0), Scalar(255));
    Mat cutImg;//裁剪延展的图像
    Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)).copyTo(cutImg);
    dstBw = srcBw | (~cutImg);
}

std::vector<double> Pallet::CalHu(cv::Mat dst)
{
    std::vector<double> Hu;
    IplImage *ssrc;
    IplImage temp = (IplImage)dst;
    ssrc = &temp;
    CvMoments moment;
    CvHuMoments humoment;
    cvMoments(ssrc,&moment);
    cvGetHuMoments(&moment, &humoment);
    if(Hu_tran_flag){
        humoment.hu1 = -1 * copysign(1.0, humoment.hu1) * log10(abs(humoment.hu1));  
        humoment.hu2 = -1 * copysign(1.0, humoment.hu2) * log10(abs(humoment.hu2)); 
        humoment.hu3 = -1 * copysign(1.0, humoment.hu3) * log10(abs(humoment.hu3)); 
        humoment.hu4 = -1 * copysign(1.0, humoment.hu4) * log10(abs(humoment.hu4)); 
        humoment.hu5 = -1 * copysign(1.0, humoment.hu5) * log10(abs(humoment.hu5)); 
        humoment.hu6 = -1 * copysign(1.0, humoment.hu6) * log10(abs(humoment.hu6)); 
        humoment.hu7 = -1 * copysign(1.0, humoment.hu7) * log10(abs(humoment.hu7));
    }
    // std::cout<<humoment.hu1<<std::endl
    //         <<humoment.hu2<<std::endl
    //         <<humoment.hu3<<std::endl
    //         <<humoment.hu4<<std::endl
    //         <<humoment.hu5<<std::endl
    //         <<humoment.hu6<<std::endl
    //         <<humoment.hu7<<std::endl
    //         <<"----------"<<std::endl;
    Hu.push_back(humoment.hu1);
    Hu.push_back(humoment.hu2);
    Hu.push_back(humoment.hu3);
    Hu.push_back(humoment.hu4);
    Hu.push_back(humoment.hu5);
    Hu.push_back(humoment.hu6);
    Hu.push_back(humoment.hu7);

    return Hu;   
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Pallet::pipline(PointTypeCloudPtr original_cloud_ptr) 
{
    // remove NAN
    PointTypeCloudPtr preprocessed_cloud_ptr(new PointTypeCloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*original_cloud_ptr, *preprocessed_cloud_ptr,
                                indices);
    PointTypeCloudPtr obstacle_cloud_ptr(new PointTypeCloud);
    std::vector<PointTypeCloudPtr> cloud_cluster;
    ground_segmenter.segment(*preprocessed_cloud_ptr, cloud_cluster);

    if (cloud_cluster.size() > 0) 
        obstacle_cloud_ptr = cloud_cluster.at(1);
    return obstacle_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Pallet::Filter_cloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PassThrough<pcl::PointXYZ> passthrough)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cache_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    passthrough.setInputCloud(cloud);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(pass_z_min,pass_z_max);//设置直通滤波器操作范围(5-12)
    passthrough.setFilterLimitsNegative(false);//false表示保留范围内，true表示保留范围外
    passthrough.filter(*cache_cloud);
    // passthrough.setInputCloud(cache_cloud);
    // passthrough.setFilterFieldName("y");
    // passthrough.setFilterLimits(-1,0.5);//设置直通滤波器操作范围
    // passthrough.setFilterLimitsNegative(true);//false表示保留范围内，true表示保留范围外
    // passthrough.filter(*filter_cloud);
    return cache_cloud;
    // return filter_cloud;
}


const std::vector<int> Pallet::Cluster_Colour(int n)
{
    std::vector<int> colour;
	switch (n)
	{
	case 0:colour={0,0,255};return colour;break;//blue
	case 1:colour={255,0,0};return colour;break;//red
	case 2:colour={0,255,0};return colour;break;//green
	case 3:colour={135,206,250};return colour;break;//LightSkyBlue
	case 4:colour={192,255,62};return colour;break;//OliveDrab1 
	case 5:colour={0,255,127};return colour;break;//SpringGreen1 	
	case 6:colour={255,246,143};return colour;break;//Khaki1 	
	case 7:colour={255,106,106};return colour;break;//IndianRed1 	
	case 8:colour={255,160,122};return colour;break;//LightSalmon1 	
	case 9:colour={218,112,214};return colour;break;//Orchid 	
	case 10:colour={255,255,255};return colour;break;//white
	default:return colour={255,255,255};return colour;
	}
}


std::vector<pcl::PointIndices> Pallet::RegionGrow_extract(
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    int show_flag)
{
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(region_mincluster);  // 最小点簇的点数
    reg.setMaxClusterSize(region_maxcluster); // 最大点簇的点数
    reg.setSearchMethod(tree); //设置点云的搜索机制
    reg.setNumberOfNeighbours(30); // 设置领域的数量
    reg.setInputCloud(cloud);  //设置输入点云
    reg.setInputNormals(cloud_normals); //设置输入法线
    reg.setSmoothnessThreshold(region_angle / 180.0 * M_PI);  // 法线角度差,设置平滑阈值大小
    reg.setCurvatureThreshold(region_curve);  // 设置曲率阈值
    reg.extract(cluster_indices);
    if(show_flag==1)
    {
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
        pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        viewer.showCloud (colored_cloud);
        while (!viewer.wasStopped ()){}
    }
    return cluster_indices;
}


pcl::PointCloud<pcl::Normal> Pallet::NormalEstimationPCA(
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne; //估计法线
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // std::cout<<"Input cloud has "<<cloud->points.size()<<" points"<<std::endl;
    ne.setInputCloud (cloud);
    //创建一个空的kdtree对象，并把它传递给法线估计对象
    ne.setSearchMethod (tree); //基于给出的输入数据集，kdtree将被建立
    pcl::PointCloud<pcl::Normal> cloud_normals; //输出数据集
    ne.setRadiusSearch(pca_rad); //使用半径在查询点周围3厘米范围内的所有邻元素
    ne.setKSearch(pca_ksearch); //计算特征值
    ne.compute (cloud_normals);
    if(cloud_normals.points.size()!=cloud->points.size())
    {
        std::cout<<"Normal Estimation Wrong!!!"<<std::endl;
        // cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同尺寸
    }
    return cloud_normals;
}

// 输入:目标点云法线信息的 object_normal
// 输出:这些目标平均法向量,以PointXYZ形式存储
//     只有z方向的法线被强制为负数,即指向传感器方向
std::vector<pcl::PointXYZ> Pallet::CalNormals(
    std::vector<pcl::PointCloud<pcl::Normal>> object_normal)
{
    std::vector<pcl::PointXYZ> Normals;
    pcl::PointXYZ normal;
    for(int i=0;i<object_normal.size();i++)
    {
        normal.x = 0;
        normal.y = 0;
        normal.z = 0;
        for(int ix=0;ix<object_normal[i].points.size();ix++)
        {
            if(std::isnan(object_normal[i].points[ix].normal_x)||
            std::isnan(object_normal[i].points[ix].normal_y)||
            std::isnan(object_normal[i].points[ix].normal_z)) {
                continue;}
            else{
                normal.x+=(object_normal[i].points[ix].normal_x);
                normal.y+=(object_normal[i].points[ix].normal_y);
                normal.z+=std::fabs(object_normal[i].points[ix].normal_z);
            }
        }
        normal.x =  (normal.x / double(object_normal[i].points.size()));
        normal.y =  (normal.y / double(object_normal[i].points.size()));
        normal.z = - (normal.z / double(object_normal[i].points.size()));
        Normals.push_back(normal);
    }
    return Normals;
}


void Pallet::Show_Normals(
    std::vector<pcl::PointCloud<pcl::PointXYZ>> object_cluster,
    std::vector<pcl::PointCloud<pcl::Normal>> object_normal)
{
    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    char label[10];
    if(object_cluster.size()!=object_normal.size())
        std::cout<<"ERROR!"<<std::endl;
    else{
        for(int i=0;i<object_cluster.size();i++)
        {
            sprintf(label,"vis-%u",i);
            viewer.addPointCloud<pcl::PointXYZ>(object_cluster[i].makeShared(),label);
            sprintf(label,"nor-%u",i);
            viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(
                object_cluster[i].makeShared(), 
                object_normal[i].makeShared(),
                1,0.03,label);
        }
        while (!viewer.wasStopped())
            {viewer.spinOnce();}
    }
}


// 输入:原始点云,passthrough,kdtree
// 中间:对原始点云进行滤波,地面去除,PCA法线估计,区域生长,再次法线估计
// 输出:存储有数个目标点云信息的 object_cluster
//     存储有这些目标点云法线信息的 object_normal
// 参数:show_flag(1:显示区域生长分割结果/2:显示法线)
void Pallet::Pallet_Extract_collect(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PassThrough<pcl::PointXYZ> passthrough,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,
    int normal_flag=0)
{
    filter_cloud = Filter_cloud(cloud,passthrough);
    offland_cloud = pipline(filter_cloud);
    cloud_normals = NormalEstimationPCA(tree,offland_cloud).makeShared();
    std::vector<pcl::PointIndices> cluster_indices;//创建点云索引向量，用于存储实际的点云索引信息，每个检测到的点云聚类都有一个索引实例
    cluster_indices = RegionGrow_extract(tree,cloud_normals,offland_cloud,show_flag);
    std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
    // std::cout << "First cluster has " << cluster_indices[0].indices.size () << " points." << endl;
    char label[10];
    int num=cluster_indices.size();
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("extract"));
    viewer->setBackgroundColor (0.0, 0.0, 0.0);
    if(num==0)//整个视野内没有一个可聚类的点云
    {
        std::cout<<"Cloud has no object"<<std::endl;
        error_flag = 1;
    }
    else
    {
        object_cluster.clear();
        int j = 1;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ> cloud_cache_cluster; 
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cache_cluster.points.push_back (offland_cloud->points[*pit]); //*
            cloud_cache_cluster.width = cloud_cache_cluster.points.size ();
            cloud_cache_cluster.height = 1;
            cloud_cache_cluster.is_dense = true;
            object_cluster.push_back(cloud_cache_cluster);
            colour=Cluster_Colour(j);
            sprintf(label,"%u-%u",1,j);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_cache_cluster.makeShared(), colour[0], colour[1], colour[2]);
            viewer->addPointCloud<pcl::PointXYZ>(cloud_cache_cluster.makeShared(),single_color,label);
            j++;
        }
        if(error_flag!=1){
            while(!viewer->wasStopped())
            {viewer->spinOnce();}}
        viewer->close();
    }

    for(int i=0;i<num;i++)
    {
         pcl::PointCloud<pcl::Normal> normals;
         normals = NormalEstimationPCA(tree,object_cluster[i].makeShared());
         object_normal.push_back(normals);
    }
    if(object_cluster.size()!=object_normal.size())
        {std::cout<<"Number ERROR!"<<std::endl;
         error_flag = 1;}
    else{
        object_avgnormal = CalNormals(object_normal);
        // std::cout<<object_avgnormal[0].x<<","
        //          <<object_avgnormal[0].y<<","
        //          <<object_avgnormal[0].z<<std::endl;
        }
    if(normal_flag&&error_flag!=1)
        Show_Normals(object_cluster,object_normal);

}


void Pallet::Pallet_Extract_match(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PassThrough<pcl::PointXYZ> passthrough,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
    filter_cloud = Filter_cloud(cloud,passthrough);
    offland_cloud = pipline(filter_cloud);
    cloud_normals = NormalEstimationPCA(tree,offland_cloud).makeShared();
    std::vector<pcl::PointIndices> cluster_indices;//创建点云索引向量，用于存储实际的点云索引信息，每个检测到的点云聚类都有一个索引实例
    cluster_indices = RegionGrow_extract(tree,cloud_normals,offland_cloud,show_flag);
    std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
    // std::cout << "First cluster has " << cluster_indices[0].indices.size () << " points." << endl;
    int num=cluster_indices.size();
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("extract"));
    viewer->setBackgroundColor (0.0, 0.0, 0.0);
    char label[10];
    if(num==0)//整个视野内没有一个可聚类的点云
    {
        std::cout<<"Cloud has no object"<<std::endl;
        error_flag = 1;
    }
    else
    {
        object_cluster.clear();
        int j = 1;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ> cloud_cache_cluster; 
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cache_cluster.points.push_back (offland_cloud->points[*pit]); //*
            cloud_cache_cluster.width = cloud_cache_cluster.points.size ();
            cloud_cache_cluster.height = 1;
            cloud_cache_cluster.is_dense = true;
            object_cluster.push_back(cloud_cache_cluster);
            colour=Cluster_Colour(j);
            sprintf(label,"%u-%u",1,j);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_cache_cluster.makeShared(), colour[0], colour[1], colour[2]);
            viewer->addPointCloud<pcl::PointXYZ>(cloud_cache_cluster.makeShared(),single_color,label);
            j++;
        }
        if(error_flag!=1){
            while(!viewer->wasStopped())
            {viewer->spinOnce();}}
        viewer->close();
    }
    for(int i=0;i<num;i++)
    {
         pcl::PointCloud<pcl::Normal> normals;
         normals = NormalEstimationPCA(tree,object_cluster[i].makeShared());
         object_normal.push_back(normals);
    }
    if(object_cluster.size()!=object_normal.size())
        {std::cout<<"Number ERROR!"<<std::endl;
         error_flag = 1;}
    else{
        object_avgnormal = CalNormals(object_normal);
        // std::cout<<object_avgnormal[0].x<<","
        //          <<object_avgnormal[0].y<<","
        //          <<object_avgnormal[0].z<<std::endl;
        }
}


// 输入:目标点云信息的 object_cluster
// 输出:目标栅格图像信息的 object_grid
void Pallet::Pallet_Grid(std::vector<pcl::PointCloud<pcl::PointXYZ>> object_cluster)
{
    for(int i=0;i<object_cluster.size();i++)
    {
        cv::Mat grid_map(map_height_, map_width_, CV_8UC1, cv::Scalar::all(0));
        GridMap.Generate(object_cluster[i].makeShared(), 255, &grid_map);
        cv::Mat dst;
        FillHole(grid_map,dst);
        if (dst.type() != CV_8UC1)    
            cv::cvtColor(dst, dst, CV_BGR2GRAY);
        cv::threshold(dst,dst,254,255,CV_THRESH_BINARY);
        object_grid.push_back(dst);
        // cv::imwrite("/media/cyber-z/E/test/test.png",grid_map);
    }
}

void Pallet::Read_FeatureModel()
{
    std::ifstream f;
    std::vector<double> cache;
    object_feature_model.clear();
    double d;
    f.open(load_feature_path);
    if(f)
    {
        for(int i=0;i<model_number;i++)
        {
            cache.clear();
            for(int j=0;j<Feature_num;j++)
            {
                f>>d;
                cache.push_back(d);
            }
            if(cache.size()!=Feature_num)
            {
                error_flag = 1;
                break;
            }
            else
                object_feature_model.push_back(cache);
        }
    }
    else
    {
        std::cout<<"Input Feature DONOT EXIST!"<<std::endl;
        error_flag = 1;
    }
    if(object_feature_model.size()!=model_number)
    {
        std::cout<<"Input Feature ERROR!"<<std::endl;
        error_flag = 1;
    }
    if(!error_flag)
        std::cout<<"Feature Model Load..."<<std::endl;
}



void Pallet::Read_ImgModel(std::string load_path)
{
    char label[20];
    std::string filename;
    object_img_model.clear();
    for(int i=0;i<model_number;i++)
    {
        cv::Mat img;
        sprintf(label,"%u.png",i);
        filename = load_path + label;
        img = cv::imread(filename);
        if(!img.data){
            std::cout<<"Input Img ERROR!"<<std::endl;
            error_flag = 1;
            break;
        }
        else
            object_img_model.push_back(img);
    }
    if(!error_flag)
        std::cout<<"Img Model Load..."<<std::endl;
}


double Pallet::Pallet_judge(cv::Mat object_img)
{
    double correlation;
    double min_correlation = 100;
    for(int i=0;i<model_number;i++)
    {
        correlation = cv::matchShapes(object_img, object_img_model[i], CONTOURS_MATCH_I2, 0);
        if(correlation<=min_correlation)
            min_correlation = correlation;
    }
    return min_correlation;
}


double Pallet::Pallet_judge(double Feature[])
{
    double correlation;
    double min_correlation = 100;
    for(int i=0;i<model_number;i++)
    {
        correlation = sqrt(
            pow((Feature[0]-object_feature_model[i][0]),2)+
            pow((Feature[1]-object_feature_model[i][1]),2)+
            pow((Feature[2]-object_feature_model[i][2]),2)+
            pow((Feature[3]-object_feature_model[i][3]),2));
        // std::cout<<correlation<<std::endl;
        if(correlation<=min_correlation)
            min_correlation = correlation;
    }
    return min_correlation;
}


void Pallet::Pallet_Match(std::vector<cv::Mat> object_grid,bool morphology,int model)
{
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    match_number = 100;
    get_flag = 0;
    double min_correlation = 100;
    double correlation;
    int min_number;
    for(int i=0;i<object_grid.size();i++)
    {
        if(object_grid[i].empty())
            std::cout<<"empty grid"<<std::endl;
        contours.clear();
        Mat canny_image;
        if(morphology)
        {
            morphologyEx(object_grid[i], object_grid[i], MORPH_CLOSE, element);
            morphologyEx(object_grid[i], object_grid[i], MORPH_OPEN, element);
        }
        if(model==1) // canny边缘+Hu+S1/S2
        {
            cv::Canny(object_grid[i],canny_image,1,3,3,false);
            cv::findContours(canny_image, contours, cv::RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
            // std::cout<<contours.size()<<std::endl;
            if(contours.size()!=1)  //取最大的?
            {
                std::vector<cv::Point> cache;
                double max_length = 0;
                double length;
                for(int i=0;i<contours.size();i++)
                {
                    length = cv::arcLength(contours[i],0);
                    if(length>max_length)
                    {
                        max_length = length;
                        cache = contours[i];
                    }
                }
                contours.clear();
                contours.push_back(cache);
            }
            cv::Rect rect = cv::boundingRect(contours[0]);
            double area = cv::contourArea(contours[0]);
            double length = cv::arcLength(contours[0],0);
            // std::cout<<rect.height<<std::endl
            //         <<rect.width<<std::endl
            //         <<area<<std::endl
            //         <<length<<std::endl;
            double S1 = rect.width / rect.height;
            double S2 = area / (rect.width*rect.height);
            // cv::rectangle(grid_map,rect,cv::Scalar(0,255,0),2,LINE_8);
            Hu = CalHu(canny_image);
            Feature[0] = Hu[0];
            Feature[1] = Hu[1];
            Feature[2] = S1;
            Feature[3] = S2;
            // std::cout<<Feature[0]<<" "<<Feature[1]<<" "<<Feature[2]<<" "<<Feature[3]<<std::endl;
            correlation = Pallet_judge(Feature);
        }
        else if(model==2)
        {
            correlation = Pallet_judge(object_grid[i]);
        }

        else if(model==3)
        {
            cv::Canny(object_grid[i],canny_image,1,3,3,false);
            correlation = Pallet_judge(canny_image);
        }
        else
        {
            std::cout<<"Model ERROR!"<<std::endl;
            break;
        }

        if(correlation<min_correlation){
            min_correlation = correlation;
            min_number = i;
        }
    }
    std::cout<<"min_correlation: "<<min_correlation<<std::endl;
    if((model == 1 && min_correlation <= Feature_Threshold)||
       (model == 2 && min_correlation <= Img_Threshold) ||
       (model == 3 && min_correlation <= Img_Threshold))
    {
        get_flag = 1;
        std_msgs::Bool pallet_get_;
        pallet_get_.data = get_flag;
        pallet_get_publisher.publish(pallet_get_);
        get_flag = 0;
        match_number = min_number;
        std::cout<<"Get Pallet!"<<std::endl;
        // object_cluster[match_number]
    }

    else
    {
        std::cout<<"No Pallet!"<<std::endl;
        std_msgs::Bool pallet_get_;
        pallet_get_.data = get_flag;
        pallet_get_publisher.publish(pallet_get_);
    }
}


void Pallet::Pallet_Collect(std::vector<cv::Mat> object_grid,bool morphology)
{
    char label[20];
    std::string filename;
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));	
    for(int i=0;i<object_grid.size();i++)
    {
        if(!object_grid[i].data) continue;
        sprintf(label,"%u-%u",frame,i);
        contours.clear();
        Mat canny_image;
        if(morphology)
        {
            morphologyEx(object_grid[i], object_grid[i], MORPH_CLOSE, element);
            morphologyEx(object_grid[i], object_grid[i], MORPH_OPEN, element);
        }
        sprintf(label,"%u-%u-grid.png",frame,i);
        filename = save_img_path+label;
        cv::imwrite(filename,object_grid[i]);

        cv::Canny(object_grid[i],canny_image,1,3,3,false);
        sprintf(label,"%u-%u-canny.png",frame,i);
        filename = save_img_path+label;
        cv::imwrite(filename,canny_image);
        cv::findContours(canny_image, contours, cv::RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());

        if(contours.size()!=1)  //取最大的?
        {
            std::vector<cv::Point> cache;
            double max_length = 0;
            double length;
            for(int i=0;i<contours.size();i++)
            {
                length = cv::arcLength(contours[i],0);
                if(length>max_length)
                {
                    max_length = length;
                    cache = contours[i];
                }
            }
            contours.clear();
            contours.push_back(cache);
        }
        cv::Rect rect = cv::boundingRect(contours[0]);
        double area = cv::contourArea(contours[0]);
        double length = cv::arcLength(contours[0],0);
        // std::cout<<rect.height<<std::endl
        //         <<rect.width<<std::endl
        //         <<area<<std::endl
        //         <<length<<std::endl;
        double S1 = rect.width / rect.height;
        double S2 = area / (rect.width*rect.height);
        // cv::rectangle(grid_map,rect,cv::Scalar(0,255,0),2,LINE_8);
        Hu = CalHu(canny_image);
        Feature[0] = Hu[0];
        Feature[1] = Hu[1];
        Feature[2] = S1;
        Feature[3] = S2;
        std::ofstream f;
        sprintf(label,"%u-%u-feature.txt",frame,i);
        filename = save_feature_path+label;
        f.open(filename.c_str(),ios::app);
        f<<setiosflags(ios::fixed)<<setprecision(2);
        f<<Hu[0]<<" "<<Hu[1]<<" "<<S1<<" "<<S2<<std::endl;
        std::cout<<filename<<" complished...."<<std::endl;
        f.close();
    }
    frame++;
}

void Pallet::Pallet_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud)
{
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    // }
    if(model=="mo-c"||model=="no-c")
    {
        Pallet_Extract_collect(original_cloud,passthrough,tree,0);
        if(error_flag==1)
        {
            std::cout<<"ERROR!!"<<std::endl;
            return;
        }
        Pallet_Grid(object_cluster);
        if(model=="mo-c")
            Pallet_Collect(object_grid,1);
        else
            Pallet_Collect(object_grid,0);
    }
    else{
        Pallet_Extract_match(original_cloud,passthrough,tree);
        if(error_flag==1)
        {
            std::cout<<"ERROR!!"<<std::endl;
            return;
        }
        Pallet_Grid(object_cluster);
        if(model=="mo-1")
            Pallet_Match(object_grid,1,1);
        else if(model=="mo-2")
            Pallet_Match(object_grid,1,2);
        else if(model=="mo-3")
            Pallet_Match(object_grid,1,3);
        else if(model=="no-1")
            Pallet_Match(object_grid,0,1);
        else if(model=="no-2")
            Pallet_Match(object_grid,0,2);
        else if(model=="no-3")
            Pallet_Match(object_grid,0,3);
        else
            std::cout<<"Model ERROR!"<<std::endl;
    }
    return;
}

void Pallet::Tof_callback(const sensor_msgs::ImageConstPtr& msg)
{

}