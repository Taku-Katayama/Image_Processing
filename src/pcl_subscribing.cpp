// ROS Header
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// C++ Header
#include <iostream>

// Public
ros::Publisher pub;

// Prototype
void callback();

// ros::Subscriber callback function
void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg(*msg, cloud);
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    
    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, *coefficients);
    
    
    if(inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given data" << std::endl;
    }
    else
    {
        for (size_t i = 0; i < inliers->indices.size (); ++i) {
         //std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "        << cloud.points[inliers->indices[i]].y << " " << cloud.points[inliers->indices[i]].z << std::endl;
        cloud.points[inliers->indices[i]].r = 255;
        cloud.points[inliers->indices[i]].g = 0;
        cloud.points[inliers->indices[i]].b = 0;
        cloud.points[inliers->indices[i]].a = 255;
        }
    }
    
    // Publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    pub.publish(output);
}

// Main function
int main(int argc, char** argv)
{
    std::cout << "--- Start !! ---" << std::endl;

    std::cout << "--- Setting ROS Node ... ";
    ros::init(argc, argv, "sub_pcl");
    std::cout << "Success" << std::endl;

    std::cout << "--- Setting ROS Handle ... ";
    ros::NodeHandle nh;
    std::cout << "Success" << std::endl;

    std::cout << "--- Subscribing Point Cloud ... ";
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud/cloud_registered", 1, callback);
    std::cout << "Success" << std::endl;
    
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    
    ros::spin();

    std::cout << "--- Finish !! ---" << std::endl;
    return 0;
}
