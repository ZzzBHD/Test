#include "pallet_top.h"
// #include <unistd.h>
using namespace cv;


int main(int argc, char ** argv)
{
  ros::init(argc,argv,"object_node");
  ros::NodeHandle node_handle;
  Pallet Pallet_zb(node_handle);
  Pallet_zb.Init();
  Pallet_zb.Read_FeatureModel();
  // Pallet_zb.Read_ImgModel(Pallet_zb.load_img_path);
  
  char filename[100];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  sprintf(filename,"/media/cyber-z/E/cache/pcd/32.pcd");
  pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud);
  for(int i=0;i<10;i++)
  {
    sprintf(filename,"/media/cyber-z/E/cache/pcd/%u.pcd",i);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
          break;
    else
    {
      Pallet_zb.Pallet_detection(cloud);
    }
  }
  Pallet_zb.Pallet_detection(cloud);

  return 0;
}