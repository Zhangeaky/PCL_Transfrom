#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector>
#include <string>

using namespace std;

typedef pcl::PointXYZ PointT;

int
main (int argc, char** argv)
{

  vector<pcl::PointCloud<PointT>::Ptr> sets(6);


    //读取点云
  pcl::PCDReader reader;
 
  pcl::PointCloud<PointT>::Ptr view1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr view2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr view3 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr view4 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr view5 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr view6 (new pcl::PointCloud<PointT>);

  sets.push_back(view1);
  sets.push_back(view2);
  sets.push_back(view3);
  sets.push_back(view4);
  sets.push_back(view5);
  sets.push_back(view6);
       
  reader.read ("/home/zhangeaky/data/pcd/view1.pcd", *view1);
  reader.read ("/home/zhangeaky/data/pcd/view2.pcd", *view2);
  reader.read ("/home/zhangeaky/data/pcd/view3.pcd", *view3);
  reader.read ("/home/zhangeaky/data/pcd/view4.pcd", *view4);
  reader.read ("/home/zhangeaky/data/pcd/view5.pcd", *view5);
  reader.read ("/home/zhangeaky/data/pcd/view6.pcd", *view6);



  //直通滤波模型
  pcl::PassThrough<PointT> passThrough;
  //法线估计模型
  pcl::NormalEstimation<PointT, pcl::Normal> ne;

  //分割对象
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;

  //对象提取
  //提取点
  pcl::ExtractIndices<PointT> extract;
  //提取法向量
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());


  // Datasets
  

  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  

  // Build a passthrough filter to remove spurious NaNs

  for(size_t i = 0; i < sets.size(); ++i){

    passThrough.setInputCloud (sets[i]);
    passThrough.setFilterFieldName ("z");
    passThrough.setFilterLimits (0, 1.5);

    //按照条件过滤的模型
    passThrough.filter (*cloud_filtered);
    string order = to_string(i);

    pcl::io::savePCDFileASCII("/home/filter/f"+order+".pcd",*cloud_filtered);


    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

      // Create the segmentation object for the planar model and set all the parameters
    // seg.setOptimizeCoefficients (true);
    // seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    // seg.setNormalDistanceWeight (0.1);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (100);
    // seg.setDistanceThreshold (0.03);
    // seg.setInputCloud (cloud_filtered);
    // seg.setInputNormals (cloud_normals);
    // // Obtain the plane inliers and coefficients
    // seg.segment (*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;






  }


  // // Extract the planar inliers from the input view1
  // extract.setInputCloud (cloud_filtered);
  // extract.setIndices (inliers_plane);
  // extract.setNegative (false);

  // // Write the planar inliers to disk
  // pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  // extract.filter (*cloud_plane);
  // std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  // writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // // Remove the planar inliers, extract the rest
  // extract.setNegative (true);
  // extract.filter (*cloud_filtered2);
  // extract_normals.setNegative (true);
  // extract_normals.setInputCloud (cloud_normals);
  // extract_normals.setIndices (inliers_plane);
  // extract_normals.filter (*cloud_normals2);

  // // Create the segmentation object for cylinder segmentation and set all the parameters
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_CYLINDER);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setNormalDistanceWeight (0.1);
  // seg.setMaxIterations (10000);
  // seg.setDistanceThreshold (0.05);
  // seg.setRadiusLimits (0, 0.1);
  // seg.setInputCloud (cloud_filtered2);
  // seg.setInputNormals (cloud_normals2);

  // // Obtain the cylinder inliers and coefficients
  // seg.segment (*inliers_cylinder, *coefficients_cylinder);
  // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // // Write the cylinder inliers to disk
  // extract.setInputCloud (cloud_filtered2);
  // extract.setIndices (inliers_cylinder);
  // extract.setNegative (false);
  // pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  // extract.filter (*cloud_cylinder);
  // if (cloud_cylinder->points.empty ()) 
  //   std::cerr << "Can't find the cylindrical component." << std::endl;
  // else
  // {
	//   std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	//   writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  // }
  // return (0);
}
