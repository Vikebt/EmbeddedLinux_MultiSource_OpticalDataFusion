#ifndef GNSS_OUTPUT_H
#define GNSS_OUTPUT_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>

#include <fstream>
#include <ctime>
#include <iomanip>
#include <csignal>
#include <thread>
#include <sstream>
#include <string>
#include <mutex>

// extern int gnss_flag;

// /***11.14.20:01 by zc***/
// extern std::string gps_time_str;  // 全局变量，用于保存GPS时间
// /***11.14.20:01 by zc***/

struct Gnss_result
{
    int mode;
    std::string gps_time;
    double roll;
    double pitch;
    double yaw;
    double altitude;
    double latitude; 
    double longitude;
    double speed;
    double speed_x;
    double speed_y;
    double speed_z;
    double imu_acc_x;
    double imu_acc_y;
    double imu_acc_z;
    double imu_gyro_x;
    double imu_gyro_y;
    double imu_gyro_z;
};

class GnssOutput
{
public:
    GnssOutput(ros::NodeHandle& nh);
    ~GnssOutput();
    void start();  // 启动同步和数据记录功能
    void start1();  // 启动同步和数据记录功能
private:
    
    std::ostringstream file_name;
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu;
    message_filters::Subscriber<sensor_msgs::NavSatFix> sub_fix;
    message_filters::Subscriber<geometry_msgs::TwistStamped> sub_vel;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::NavSatFix, geometry_msgs::TwistStamped> HighFreqSyncPolicy;

    //message_filters::Synchronizer<FullSyncPolicy> sync;
    message_filters::Synchronizer<HighFreqSyncPolicy> high_freq_sync;

    // bool high_freq_triggered;  // 标志变量，控制高频和低频输出

    // void callback(const sbg_driver::SbgEkfEulerConstPtr& euler_data, 
    //               const sbg_driver::SbgGpsPosConstPtr& gps_data,
    //               const sbg_driver::SbgEkfNavConstPtr& ekf_nav_data,
    //               const sbg_driver::SbgUtcTimeConstPtr& utc_time_data);

    void highFreqCallback(const sensor_msgs::ImuConstPtr& imu_msg, 
                          const sensor_msgs::NavSatFixConstPtr& fix_msg,
                          const geometry_msgs::TwistStampedConstPtr& vel_msg);

    
    static void signalHandler(int signum);
public:
    int gnss_flag; // 用于标记GNSS状态的全局变量
    std::mutex gnss_flag_mutex;

    std::string gps_time_str;  // 用于保存GPS时间
    //std::string message;//GNSS输出的所有信息

    int num;//获取gnss信息的数量

    Gnss_result result;

    int capture_num;//抓取的信息数量

};

#endif // GNSS_OUTPUT_H