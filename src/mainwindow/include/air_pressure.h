#ifndef AIR_PRESSURE_NODE_H
#define AIR_PRESSURE_NODE_H

#include <iostream>
#include <fstream>
#include <string>
#include <serial/serial.h>
#include <cstdlib>  // for strtol
#include <cstring>  // for memcpy
#include <thread>

struct AirPressure_result
{
    std::string gps_time_str;
    float pressure_a;//大气静压
    float altitude_a;//标准气压高度
    float rate;//爬升率
    float altitude_b;//气压校正高度
};


class AirPressureSensor
{
public:
    AirPressureSensor(const std::string& port, int baudrate);
    ~AirPressureSensor();
    // 解析数据
    bool record_air_pressure();
    //初始化串口
    bool initializeSerial();

    //std::string data;//解码数据
    
    AirPressure_result mresult;
    std::string gps_time_str;

    //抓取串口数据线程
    std::unique_ptr<std::thread> capture_thread;
    void capture_serial_thread();
    bool capture_thread_flag;
    //开始线程运行
    void start_thread();
    //停止线程
    void stop_thread();

private:
    // 串口相关
    serial::Serial ser;
    std::string port_;
    int baudrate_;

    // 缓冲区
    std::vector<unsigned char> r_buffer;

    // 帧头
    unsigned char headerFrame[7];
    
    // 解码工具函数
    float decodeHexToFloat(const char* hexData);

    // 检查是否有匹配的开头帧
    bool checkHeaderFrame(const std::vector<unsigned char> buffer, size_t bufferSize);

    // 解码数据并格式化输出
    void decodeAndFormatData(size_t available_data);

    //是否是第一帧
    bool is_first_frame;
    //收到的第一帧数据的高度
    float initial_height;

};

#endif // AIR_PRESSURE_NODE_H