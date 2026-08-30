#ifndef SENSOR_DATA_RECEIVER_H
#define SENSOR_DATA_RECEIVER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector> // 必须包含 vector
#include <thread>
#include <mutex>  // 推荐加上锁保护数据
#include <serial/serial.h>

struct Radio_result
{
    float altitude;
    std::string gps_time;
};

class SensorDataReceiver
{
public:
    SensorDataReceiver(const std::string &port, uint32_t baud_rate);
    ~SensorDataReceiver();

    // 线程主循环调用：读取数据、解码、更新结果
    void record_radio_altitude();

    // 关闭串口
    void closeSerial();
    
    // 初始化串口
    bool initializeSerial();

    // 对外暴露的结果
    Radio_result result;
    std::string gps_time_str; // 注意：需要外部或其他逻辑来更新这个时间字符串

    // 抓取串口数据线程管理
    void start_thread();
    void stop_thread();

private:
    // 串口对象
    serial::Serial ser;
    std::string port_;
    uint32_t baud_rate_;

    //流式处理核心缓冲区
    std::vector<uint8_t> m_buffer; 

    // 线程相关
    std::unique_ptr<std::thread> capture_thread;
    bool capture_thread_flag;
    void capture_serial_thread();

    // --- 内部辅助函数 ---

    // 从串口读取数据并追加到缓冲区，尝试解析
    // 返回值：true 表示解析到了新数据，output_height 为最新高度
    // 返回值：false 表示缓冲区数据不足或无新数据
    bool readAndDecodeData(float &output_height);

    // 计算校验和
    uint8_t calculateChecksum(const uint8_t* frame_data, size_t length);
};

#endif  // SENSOR_DATA_RECEIVER_H