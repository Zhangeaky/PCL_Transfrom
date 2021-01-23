#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include  <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include  <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>

using namespace std;





int test01(){

  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.width  = 15;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1.0;
  }

  cloud.points[0].z = 2.0;
  cloud.points[3].z = -2.0;
  cloud.points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud.points.size () <<" points" << std::endl;
  
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " 
  << cloud.points[i].y << " " 
  << cloud.points[i].z << std::endl;

  pcl::io::savePCDFile("/home/zhangeaky/data/pcd/test01.pcd", cloud );


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients (true);

  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0){
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
  }
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  <<coefficients->values[1] << " "  
  <<coefficients->values[2] << " " 
  <<coefficients->values[3] <<std::endl;
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " <<cloud.points[inliers->indices[i]].x << " "
  <<cloud.points[inliers->indices[i]].y << " "
  <<cloud.points[inliers->indices[i]].z << std::endl;
  return (0);
}
int main(int argc, char** argv){

    ros::init(argc,argv,"filter");
    ros::NodeHandle n;

    test01();

    // string path = "";

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile(path, *cloud);

    // pcl::PointCloud<pcl::PointXYZ> outcloud;

    // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);

    // pcl::BilateralFilter<pcl::PointXYZ> bf;

    // bf.setInputCloud(cloud);
    //bf.setSearchMethod(tree);


    


}