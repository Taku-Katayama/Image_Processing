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
// Eigen
#include <Eigen/Core>

ros::Publisher pub;

// Prototype
void callback();

// ros::Subscriber callback function
void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg(*msg, cloud);
    
    for(size_t i = 0; i < cloud.size(); ++i)
    {
        if(cloud.points[i].z < 0)
        {
            cloud.points[i].r = 255;
            cloud.points[i].g = 0;
            cloud.points[i].b = 0;
            cloud.points[i].a = 255;
        }
    }
    
    //pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    //pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    //seg.setOptimizeCoefficients(true);
    // Mandatory
    //seg.setModelType(pcl::SACMODEL_PLANE);
    //seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setDistanceThreshold(0.1);
    //seg.setAxis(Eigen::Vector3f(0, 1, 0) );
    //seg.setEpsAngle(0.1);
    
    //seg.setInputCloud(cloud.makeShared());
    //seg.segment(*inliers, *coefficients);
    
    //if(inliers->indices.size() == 0)
    //{
        //std::cerr << "Could not estimate a set model for the given data" << std::endl;
    //}
    //else
    //{
        //for(size_t i = 0; i < inliers->indices.size(); ++i)
        //{
            //cloud.points[inliers->indices[i]].r = 255;
            //cloud.points[inliers->indices[i]].g = 0;
            //cloud.points[inliers->indices[i]].b = 0;
            //cloud.points[inliers->indices[i]].a = 255;
        //}
    //}
    
    // Publish the model coefficients
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
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
    
    pub = nh.advertise<sensor_msgs::PointCloud2>("/image_processing/output", 1);
    
    ros::spin();

    printf("--- Finish !! ---\n");
    return 0;
}
