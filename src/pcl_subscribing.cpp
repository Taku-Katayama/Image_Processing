#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
//#include <boost/thread/thread.hpp>

void callback();
void pclViewer();

//pcl::PointCloud<pcl::PointXYZRGB> cloud;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//pcl::PointCloud<PointType>::Ptr cloud( new pcl::PointCloud<PointType> );

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  printf("sensor_msgs::PiontCloud2: width = %d, height = %d\n", msg->width, msg->height);

  // Convert sensor_msgs::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
  pcl::fromROSMsg(*msg, *cloud);
  printf("pcl::PointCloud<pcl::PointXYZ>: width = %d, height = %d\n", cloud->width, cloud->height);
  pclViewer();
  //printf("cloud.points.size() = %lu\n",cloud.points.size());
  //printf("[x, y, z] = [%d, %d, %d]",cloud.points[0].x, cloud.y, cloud.z);
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  //pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
  //viewer.spinOnce();
  //viewer.updatePointCloud( cloud, msg );
}

void pclViewer()
{
    try
    {
        // pcl::visualization::CloudViewer viewer("Cloud Viewer");
        // viewer.showCloud(cloud);
        // 表示
        int vp[2];
        pcl::visualization::PCLVisualizer viewer("Viewer");
        pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud);
        //viewer.createViewPort(0.0, 0.0, 0.5, 1.0, vp[0]);
        viewer.createViewPort(0.0, 0.0, 1.0, 1.0, vp[1]);
        //viewer.addPointCloud(cloud, "cloud_without_color_handler", vp[0]);    // 透明度が反映されない
        viewer.addPointCloud(cloud, rgba, "cloud_with_color_handler", vp[1]);    // 透明度が反映される
        //viewer.addText("Without color handler", 10, 10, "text_without_color_handler", vp[0]);
        viewer.addText("With color handler", 10, 10, "text_with_color_handler", vp[1]);
        viewer.spin();
    }
    catch( std::exception& ex )
    {
        std::cout << ex.what() << std::endl;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud/cloud_registered", 1, callback);
  ros::spin();
}
