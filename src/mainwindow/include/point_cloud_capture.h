#ifndef POINT_CLOUD_CAPTURE_H
#define POINT_CLOUD_CAPTURE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCapture
{
public:
    PointCapture(ros::NodeHandle& nh);
    ~PointCapture();

private:
    ros::Subscriber lslidar_point_cloud_sub;      //声明一个lslidar_point_cloud话题订阅者 10hz
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);//lslidar_point_cloud_sub话题的回调函数

public:
    pcl::PointCloud<pcl::PointXYZ> now_cloud;
    std::string gps_time_str;  // 用于保存GPS时间
    int capture_num;
};

#endif 