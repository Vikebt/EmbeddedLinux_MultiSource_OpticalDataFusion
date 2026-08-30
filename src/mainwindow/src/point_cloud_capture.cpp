#include "point_cloud_capture.h"
#include "../include/point_cloud_capture.h"


PointCapture::PointCapture(ros::NodeHandle &nh):
capture_num(0)
{
    lslidar_point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 10, &PointCapture::pointcloud_callback,this);
}

PointCapture::~PointCapture()
{
    std::cout << "pointcapture析构" << std::endl;
}

void PointCapture::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // 将 ROS 点云消息转换为 PCL 点云
    pcl::fromROSMsg(*msg, now_cloud);
    capture_num ++;
}
