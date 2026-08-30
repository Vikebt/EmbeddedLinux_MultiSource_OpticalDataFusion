#include "../include/gnss_output.h"
#include <cmath>
#include "gnss_output.h"

// 构造函数：初始化订阅者和同步器
GnssOutput::GnssOutput(ros::NodeHandle& nh)
    : sub_imu(nh, "/fdi_imu", 40),
      sub_fix(nh, "/fdi_gps", 40),
      sub_vel(nh, "/fdi_enu", 40),
      high_freq_sync(HighFreqSyncPolicy(1000), sub_imu, sub_fix, sub_vel),
      gnss_flag(0),
      capture_num(0),
      num(0)
{
    high_freq_sync.registerCallback(boost::bind(&GnssOutput::highFreqCallback, this, _1, _2, _3));
}

// 析构函数：关闭文件
GnssOutput::~GnssOutput()
{
    std::cout<<"GNSS析构"<<std::endl;
}



// 高频回调函数
void GnssOutput::highFreqCallback(const sensor_msgs::ImuConstPtr& imu_msg, 
                                  const sensor_msgs::NavSatFixConstPtr& fix_msg,
                                  const geometry_msgs::TwistStampedConstPtr& vel_msg)
{
       Gnss_result temp_result;

        // 提取欧拉角 (Roll, Pitch, Yaw)
        double roll, pitch, yaw;
        // tf2::Quaternion q(
        //     imu_msg->orientation.x,
        //     imu_msg->orientation.y,
        //     imu_msg->orientation.z,
        //     imu_msg->orientation.w
        // );
        // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 转换为角度
        roll = imu_msg->orientation.x * 180.0 / M_PI;
        pitch = imu_msg->orientation.y * 180.0 / M_PI;
        yaw = imu_msg->orientation.z * 180.0 / M_PI;

        double latitude = fix_msg->latitude;
        double longitude = fix_msg->longitude;
        double altitude = fix_msg->altitude;

        double velocity_x = vel_msg->twist.linear.x;
        double velocity_y = vel_msg->twist.linear.y;
        double velocity_z = vel_msg->twist.linear.z;

        double imu_acc_x = imu_msg->linear_acceleration.x;
        double imu_acc_y = imu_msg->linear_acceleration.y;
        double imu_acc_z = imu_msg->linear_acceleration.z;

        double imu_gyro_x = imu_msg->angular_velocity.x * 180 / M_PI;
        double imu_gyro_y = imu_msg->angular_velocity.y * 180 / M_PI;
        double imu_gyro_z = imu_msg->angular_velocity.z * 180 / M_PI;

        // 处理时间 (UTC + 8)
        // 使用 imu_msg 的时间戳
        time_t raw_time = imu_msg->header.stamp.sec + 8 * 3600; 
        struct tm * time_info = gmtime(&raw_time);

        int hour = time_info->tm_hour;
        int min = time_info->tm_min;
        int sec = time_info->tm_sec;
        int nanosec = imu_msg->header.stamp.nsec;

        /***11.14.20:01 by zc***/
        // 将GPS时间格式化为字符串并保存在全局变量 gps_time_str 中
        std::ostringstream time_stream;
        time_stream << std::setw(2) << std::setfill('0') << hour << ":" 
                    << std::setw(2) << std::setfill('0') << min << ":" 
                    << std::setw(2) << std::setfill('0') << sec << "." 
                    << std::setw(3) << std::setfill('0') << nanosec/1000000;

        gps_time_str = time_stream.str();  // 保存格式化的时间字符串
        
        temp_result.mode = int(fix_msg->status.status);
        temp_result.pitch = pitch;
        temp_result.roll = roll;
        temp_result.yaw = yaw;
        temp_result.altitude = altitude;
        temp_result.latitude = latitude;
        temp_result.longitude = longitude;
        temp_result.gps_time = gps_time_str;
        temp_result.speed_x = velocity_x;
        temp_result.speed_y = velocity_y;
        temp_result.speed_z = velocity_z;
        temp_result.imu_acc_x = imu_acc_x;
        temp_result.imu_acc_y = imu_acc_y;
        temp_result.imu_acc_z = imu_acc_z;
        temp_result.imu_gyro_x = imu_gyro_x;
        temp_result.imu_gyro_y = imu_gyro_y;
        temp_result.imu_gyro_z = imu_gyro_z;
        temp_result.speed = sqrt(velocity_x * velocity_x + velocity_y * velocity_y + velocity_z * velocity_z);

        //std::cout<<"gnss:"<<gps_time_str<<std::endl;
        result = temp_result;
        capture_num++;
    
}

// 信号处理函数：确保程序退出时关闭文件
void GnssOutput::signalHandler(int signum)
{
    ROS_INFO("Closing file...");
    ros::shutdown();
}

// 启动同步功能
void GnssOutput::start()
{
    signal(SIGINT, GnssOutput::signalHandler);

    ros::AsyncSpinner spinner(2); // 启动两个线程
    spinner.start();

    while (ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 示例：保持循环运行
    }

    spinner.stop();
}


void GnssOutput::start1()
{
    signal(SIGINT, GnssOutput::signalHandler);
    // 使用单线程运行 ROS 节点
}
