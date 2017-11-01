// ROS Header
#include <ros/ros.h>
// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/filters/voxel_grid.h>
// C++ Header
#include <iostream>

ros::Publisher pub;

// Prototype
void callback();
void planeDetect(pcl::PointCloud<pcl::PointXYZRGB> cloud);

// ros::Subscriber callback function
void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    
    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);
    
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(cloud_filtered);
    
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);
    
    pub.publish(output);
        
}

// Main function
int main(int argc, char** argv)
{
    // Start Message
    printf("--- Start !! ---\n");

    // ROS Initialization
    printf("--- Setting ROS Node ... \n");
    ros::init(argc, argv, "sub_pcl"); // Node

    printf("--- Setting ROS Handle ... \n");
    ros::NodeHandle nh;               // Handle

    printf("--- Subscribing Point Cloud  ... \n"); 
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud/cloud_registered", 1, callback); // Subscriber
    
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>("output", 1);
    
    ros::spin();

    printf("--- Finish !! ---\n");
    return 0;
}
