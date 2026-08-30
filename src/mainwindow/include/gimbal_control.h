#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <locale.h>
#include <vector>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdint>
#include <iostream>
#include <thread>
#include <queue>
#include <stack> 
#include <condition_variable>
#include <yaml-cpp/yaml.h>
#include "gnss_output.h" 


// 数据发送频率（Hz）
#define SEND_RATE 1


//光电球设置的一些参数
struct gimbal_config
{
    int camera_mode;//0-可见光  1-红外
    int focus;//变焦倍数
    int yaw;//偏航
    int pitch;//俯仰
    int roll;//滚转
};

struct Gimbal_result
{
    std::string gps_time_str;
    float distance;//激光测距
    float roll;//滚转
    float pitch;//俯仰
    float yaw;//偏航
    int focus;//变焦倍数
    std::string tracking_status;//跟踪状态
    double target_latitude;//目标纬度
    double target_longitude;//目标经度
    float target_altitude;//目标高度
    std::string oss_zoom;//焦距信息
    std::string gimbal_mode_str;//云台工作模式
    uint8_t checksum;//计算的校验码
    uint8_t received_checksum;//实际的校验码
};

class GimbalController
{
public:
    GimbalController(ros::NodeHandle& nh);
    ~GimbalController();

    void sendData(const std::vector<uint8_t>& data, std_msgs::Int32& status_msg);//向串口写数据
    void decodeData(const std::vector<uint8_t>& received_data);//解析回传数据
 
    bool initializeDevice();//初始化吊舱
    bool changeAngle(int pitch_angle, int yaw_angle);//改变吊舱角度
    std::vector<uint8_t> generateAngleCommand(int angle, bool is_pitch);//生成角度指令

    //指定参数控制吊舱
    bool set_gimbal_config(gimbal_config para_);
    //控制焦距
    bool set_gimbal_zoom(bool up, int multiple);
    //控制吊舱一键朝下
    bool set_gimbal_down();
    //控制吊舱一键回中
    bool set_gimabl_center();

    ros::Publisher pub_;
    ros::Publisher status_pub_;
    serial::Serial ser_;

    Gimbal_result result;//吊舱当前状态的角度结果
    //抓取串口数据线程
    std::unique_ptr<std::thread> capture_thread;
    void capture_serial_thread();
    bool capture_thread_flag;

    //开始线程
    void start_thread();
    void stop_thread();

private:    
    ros::NodeHandle& nh_;

    std::string getCurrentTimeForFilename();//获取当前时间 未用到
    void printCurrentTime();//打印当前时间
    void saveCurrentTimeToFile(const std::string& filename);//将时间序列存下来 未用到

public:
    bool initializeSerial();
public:
    std::string GIMBAL_SERIAL_PORT;
    int GIMBAL_SERIAL_BAUDRATE;
    int PITCH_ANGLE;
    int YAW_ANGLE;
    int CAMERA_MODE;//相机模式 0:可见光 1：红外
    int FOCUS;//相机焦距 
    int previous_focus;//上一次的相机焦距
    
    int* share_cameramode;

    // 十六进制数组指令
    std::vector<uint8_t> photo_data;  
    std::vector<uint8_t> start_video_data;  
    std::vector<uint8_t> stop_video_data;  
    std::vector<uint8_t> yaw_angle_to_zero; 
    std::vector<uint8_t> visible_light_mode;
    std::vector<uint8_t> infrared_mode; 
    std::vector<uint8_t> visible_light_amplification_plus;
    std::vector<uint8_t> visible_light_amplification_minus;
    std::vector<uint8_t> visible_light_stops_doubling;
    std::vector<uint8_t> yaw_lock;
};

#endif