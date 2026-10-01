#include "point_cloud_capture.h"
#include "../include/point_cloud_capture.h"


PointCapture::PointCapture(ros::NodeHandle &nh)
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
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    now_cloud_ = std::move(cloud);
    ++capture_num_;
}

bool PointCapture::latest(pcl::PointCloud<pcl::PointXYZ>* output, int* sequence) const
{
    if (output == nullptr) return false;
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (capture_num_ == 0) return false;
    *output = now_cloud_;
    if (sequence != nullptr) *sequence = capture_num_;
    return true;
}
