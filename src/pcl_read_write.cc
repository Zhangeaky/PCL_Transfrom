#include<ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
//ros点云数据和pcl点云数据的转换
#include <pcl_ros/point_cloud.h> 
//#include <pcl/ros/conversions.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

using namespace std;

//this function is defined in: make_clouds.cpp
extern void make_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr);

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_learning_1"); //node name
    ros::NodeHandle nh;

    // create some point-cloud objects to hold data
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //no color
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //colored

    cout << "Generating example point-cloud ellipse.\n\n";
    cout << "view in rviz; choose: topic= ellipse; and fixed frame= camera" << endl;

    // -----use fnc to create example point clouds: basic and colored-----
    make_clouds(basic_cloud_ptr, point_cloud_clr_ptr);
    pcl::io::savePCDFileASCII ("ellipse.pcd", *point_cloud_clr_ptr); //save image to disk
    // we now have "interesting" point clouds in basic_cloud_ptr and point_cloud_clr_ptr
    // 将带有颜色的点云保存

    sensor_msgs::PointCloud2 ros_cloud; //here is the ROS-compatible pointCloud message
    //we'll publish the colored point cloud; 
    pcl::toROSMsg(*point_cloud_clr_ptr, ros_cloud); //convert from PCL to ROS type this way

    
    //let's publish the colored point cloud in a ROS-compatible message; 
    // we'll publish to topic "ellipse"
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ellipse", 1);

    //publish the ROS-type message; can view this in rviz on topic "/ellipse"
    //BUT need to set the Rviz fixed frame to "camera"
    while (ros::ok()) {
        pubCloud.publish(ros_cloud);
        ros::Duration(0.5).sleep(); //keep refreshing the publication periodically
    }
    return 0;
}