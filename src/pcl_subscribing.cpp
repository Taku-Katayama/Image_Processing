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
// C++ Header
#include <iostream>

ros::Publisher pub;

// Prototype
void callback();

// Global variable
//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

// ros::Subscriber callback function
void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //printf("sensor_msgs::PiontCloud2: width = %d, height = %d\n", msg->width, msg->height);

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    
    // Convert sensor_msgs::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
    pcl::fromROSMsg(*msg, cloud);

    for(size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].a = 255;
    }

    // show point cloud infomation
    //std::cout << *cloud << std::endl;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    double dist_th;
    ros::param::get("dist_th", dist_th);
    seg.setDistanceThreshold(dist_th);

    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, *coefficients);
    
    if(inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given data" << std::endl;
    }
    else
    {
        for (size_t i = 0; i < inliers->indices.size (); ++i) {
        // std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << "       // << cloud.points[inliers->indices[i]].y << " "
        // << cloud.points[inliers->indices[i]].z << std::endl;
        cloud.points[inliers->indices[i]].r = 0;
        cloud.points[inliers->indices[i]].g = 0;
        cloud.points[inliers->indices[i]].b = 255;
        cloud.points[inliers->indices[i]].a = 255;
        }
    }


    // save pcd file
    //pcl::io::savePCDFileASCII ("save.pcd", cloud);

    pub.publish(cloud);
        
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
    
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>("planar_segmentation", 1);
    
    ros::spin();

    printf("--- Finish !! ---\n");
    return 0;
}
