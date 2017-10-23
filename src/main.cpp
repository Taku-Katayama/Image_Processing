// ROS Header
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
// PCL Header
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// C++ Header
#include <iostream>

// Prototype
void callback();

// Global variable
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

// ros::Subscriber callback function
void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //printf("sensor_msgs::PiontCloud2: width = %d, height = %d\n", msg->width, msg->height);

    // Convert sensor_msgs::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
    pcl::fromROSMsg(*msg, *cloud);

    // show point cloud infomation
    std::cout << *cloud << std::endl;
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
    ros::spin();

    printf("--- Finish !! ---\n");
    return 0;
}
