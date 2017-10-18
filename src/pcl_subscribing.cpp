// ROS Header
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
// PCL Header
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// C++ Header
#include <iostream>
#include <boost/thread/thread.hpp>

// Prototype
void callback();
void pclViewer();

std::thread zed_callback;

// Global variable
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::visualization::PCLVisualizer viewer("Viewer");

int vp;
pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud);
viewer.createViewPort(0.0, 0.0, 1.0, 1.0, vp);
viewer.addPointCloud(cloud, rgba, "cloud_with_color_handler", vp);

// ros::Subscriber callback function
void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  printf("sensor_msgs::PiontCloud2: width = %d, height = %d\n", msg->width, msg->height);

  // Convert sensor_msgs::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
  pcl::fromROSMsg(*msg, *cloud);
  printf("pcl::PointCloud<pcl::PointXYZ>: width = %d, height = %d\n", cloud->width, cloud->height);
  zed_callback = std::thread(pclViewer);
  //printf("cloud.points.size() = %lu\n",cloud.points.size());
  //printf("[x, y, z] = [%d, %d, %d]",cloud.points[0].x, cloud.y, cloud.z);
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  //pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
  //viewer.spinOnce();
  //viewer.updatePointCloud( cloud, msg );
}

// PCL Visulization function
void pclViewer()
{
    try
    {
	while (!viewer.wasStopped() )
	{
            viewer.updatePointCloud( cloud );
            //viewer.spinOnce(100);
	    //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
    }
    catch( std::exception& ex )
    {
        std::cout << ex.what() << std::endl;
    }
}

// Main function
int main(int argc, char** argv)
{
  // ROS Initialization
  ros::init(argc, argv, "sub_pcl"); // Node
  ros::NodeHandle nh;               // Handle
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud/cloud_registered", 1, callback); // Subscriber
  ros::spin();
  return 0;
}
