#ifndef QNODE_H
#define QNODE_H
#include <QObject>
#include <QImage>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "yaml_config.h"
#include "gimbal_control.h"
// #include "px4_serial.h"
#include "yaml-cpp/yaml.h"
#include "rtsp_capture.h"
#include "radio_altitude.h"
#include "air_pressure.h"
#include "point_cloud_capture.h"
#include <pcl/io/pcd_io.h>
#include "threadPointcloud.h"
#include <QThread>
#include <QMutex>
#include <QQueue>


class QNode:public QObject
{
    Q_OBJECT
public:
    QNode(int argc, char** argv,QObject *parent = nullptr);
    ~QNode();
    bool init();
    

    //把图片转化为Qimage
    void cvMat2QImage(const cv::Mat &mat, QImage& image); 

    //创建文件夹
    void create_folder();
    //创建顶层文件夹
    void create_top_folder();

    //存储数据
    void save_data(cv::Mat& frame, Gimbal_result& gimabal_result, 
                    Gnss_result& gnss_result, AirPressure_result& airPressure_result, 
                    Radio_result& radio_result, pcl::PointCloud<pcl::PointXYZ>& point);
    //存储吊舱数据
    void save_gimbal_data(Gimbal_result& gimabal_result, std::string path);
    //存储GNSS数据
    void save_GNSS_data(Gnss_result& gnss_result, std::string path);
    //存储气压高度计信息
    void save_airpressure_data(AirPressure_result& airPressure_result, std::string path);
    //存储无线电高度计数据
    void save_radio_data(Radio_result& radio_result, std::string path);
    //存储点云
    void save_point_data(pcl::PointCloud<pcl::PointXYZ>& point, std::string path); 
    //获取当前时间
    std::string get_current_time_string();

signals:
    void rosShutdown();
    void sig_qnode_finished();
    void sig_update_image(QImage frame);//更新图片显示
    //void sig_update_GNSS_Message(std::string message);//更新GNSS信息
    void sig_update_GNSS_result(Gnss_result result);//更新GNSS输出结果
    void sig_update_gimbal_result(Gimbal_result result);//更新吊舱角度仪表
    void sig_update_airPressure_result(AirPressure_result result);//更新气压高度计仪表
    void sig_update_airPressure_status(std::string status);//更新气压计高度表状态
    //void sig_update_radio_altitude_message(float message);//更新无线电高度计信息
    void sig_update_radio_altitude_result(Radio_result result);//更新无线电高度计信息
    //void sig_update_air_pressure_message(std::string message);//更新气压高度计信息
    void sig_update_status_bar(QString message);//更新状态栏
    void sig_update_label_image_num(int num);//更新图数量显示
    void sig_update_label_time(QString time);//更新界面时间显示 （目前没用到）
    void sig_clear_message();//清空GNSS、气压高度计、无线电高度计显示栏，防止太多界面卡顿
    void sig_connect_succeed();//吊舱初始化成功
    void sig_update_plainTextEdit_status(QString message);//更新消息栏
    void sig_save_pts(pcl::PointCloud<pcl::PointXYZ>& point, std::string path);//存储点云
    void sig_save_pts();

public slots:
    void run();

public:
    YamlConfig yaml_config;
    YAML::Node config;//读来的参数
    int camera_mode;//共享的变量

    GimbalController* gimbal_controller = nullptr;
    GnssOutput* gnss_output = nullptr;
    RTSPCapture* rtsp_capture = nullptr;
    SensorDataReceiver* radio_altitude = nullptr;
    AirPressureSensor* airpressure_sensor = nullptr;
    PointCapture* point_capture = nullptr;
    
    //传感器初始化流程完成标志(不保证都成功)
    bool connect_succeed;
    //保存信息标志位
    bool save_flag;
    //gnss获取到的时间
    std::string gps_time_str;
    //用于控制传输状态
    bool keep_transmitting;
    //图片数量
    int img_num;
    //结束线程标志
    bool close_flag;
    //是否发送GNSS信息
    bool send_GNSS;
    //是否发送无线电高度计信息
    bool send_radio_altimeter;
    //是否发送气压高度计信息
    bool send_barometric_altimeter;

    bool connect_gimbal;//吊舱是否成功初始化
    bool connect_rtsp;//图像是否成功初始化
    bool connect_gnss;//gnss是否会成功初始化
    bool connect_radio_altitude;//无线电高度计是否成功初始化
    bool connect_air_pressure;//气压高度计是否成功初始化
    bool connect_point;//lidar是否成功初始化

    int previous_capture_num;//之前抓到的图片数量 用于判断是否抓到新的图片
    int previous_capture_num_gnss;//之前抓到的GNSS数量 用于判断是否抓到新的GNSS信息
    int previous_capture_num_point;//之前抓到的点云数量 用于判断是否抓到新的点云信息

    //ros话题
    ros::Publisher pub_radio_altitude;
    ros::Publisher pub_airpressure;
    ros::Publisher pub_image;
    ros::Publisher pub_gimabal_status;
    ros::Publisher pub_GNSS;

    //创建的保存图片的文件夹路径和串口信息路径
    std::string photo_path;
    std::string serial_path;
    std::vector<std::string> photo_serial_path;
    //std::string folder_path;//photo和serial上一级文件夹路径
    std::string top_folder_path;//photo和serial的上上一级文件夹路径


    //保存点云的线程
    QThread mThread;
    ThreadPointcloud* mThreadPointcloud;
    QQueue<pcl::PointCloud<pcl::PointXYZ>> mqueue_pts;
    QQueue<std::string> mqueue_names;
    QMutex mtx;
    
private:
    int init_argc;
    char** init_argv;

};
#endif // QNODE_H