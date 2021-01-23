#include<ros/ros.h> 
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
//ros点云数据和pcl点云数据的转换
#include <pcl_ros/point_cloud.h> 
//#include <pcl/ros/conversions.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

using namespace std;
string order;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg){

    sensor_msgs::PointCloud2 new_msgs = *msg;
    cout<<"Datasize: "<<msg->data.size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr msgptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*msgptr);

   
    string  filename ="/home/zhangeaky/room"+order+".pcd";
    pcl::io::savePCDFileASCII (filename, *msgptr); 
    cout << "Generating "<<filename<<endl;
    
    
}
   
int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_to_pcd");
    ros::NodeHandle nh;
  
    nh.getParam("/bag_to_pcd/pcd_view_order", order);
    cout<<"The order this time is: "<<order<<endl;
    
   
    ros::Subscriber sub = nh.subscribe("/kinect2/hd/points",1,callback);
    ros::spin();
    return 0;
}
