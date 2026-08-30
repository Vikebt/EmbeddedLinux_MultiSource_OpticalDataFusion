#include "../include/radio_altitude.h"
#include <chrono>
#include <thread>
#include <algorithm> // for std::min

SensorDataReceiver::SensorDataReceiver(const std::string &port, uint32_t baud_rate)
: port_(port), baud_rate_(baud_rate), capture_thread_flag(false)
{
    // 预分配缓冲区空间，避免频繁重新分配内存
    m_buffer.reserve(1024);
}

SensorDataReceiver::~SensorDataReceiver()
{
    stop_thread();
    closeSerial();
}

bool SensorDataReceiver::initializeSerial()
{
    try
    {
        ser.setPort(port_);
        ser.setBaudrate(baud_rate_);
        
        // 关键：设置偶校验（根据传感器要求）
        ser.setParity(serial::parity_even);  
        ser.setBytesize(serial::eightbits);   
        ser.setStopbits(serial::stopbits_one); 
        
        // 设置非阻塞读取或极短的超时，流式处理不需要长超时
        serial::Timeout to = serial::Timeout::simpleTimeout(100); 
        ser.setTimeout(to);
        
        ser.open();
        
        if (ser.isOpen())
        {
            std::cout << "Succeed to open radio_altitude_port: " << port_ << std::endl;
            return true;
        }
    }
    catch (serial::IOException &e)
    {
        std::cerr << "Unable to open radio_altitude_port: " << e.what() << std::endl;
    }
    catch (serial::SerialException &e)
    {
        std::cerr << "串口设置异常: " << e.what() << std::endl;
    }
    return false;
}

void SensorDataReceiver::closeSerial()
{
    if (ser.isOpen())
    {
        ser.close();
        std::cout << "Serial port closed." << std::endl;
    }
}

// 计算校验和（取低8位）
uint8_t SensorDataReceiver::calculateChecksum(const uint8_t* frame_data, size_t length)
{
    uint16_t sum = 0;
    // 校验和通常是前N个字节的累加，这里根据你之前的逻辑：
    // sum = buffer[i] + ... + buffer[i+4]
    // 传入的 frame_data 指向帧头，length 应该是 5 (不包含 checksum 本身)
    for (size_t i = 0; i < length; i++) {
        sum += frame_data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

// 核心流式处理函数
bool SensorDataReceiver::readAndDecodeData(float &output_height)
{
    // 1. 读取串口所有可用数据
    try {
        size_t available = ser.available();
        if (available > 0)
        {
            std::vector<uint8_t> temp_buf(available);
            size_t n = ser.read(temp_buf.data(), available);
            // 将新读到的数据追加到成员缓冲区末尾
            if (n > 0) {
                m_buffer.insert(m_buffer.end(), temp_buf.begin(), temp_buf.begin() + n);
            }
        }
    } catch (std::exception &e) {
        std::cerr << "串口读取异常: " << e.what() << std::endl;
        return false;
    }

    // 2. 解析缓冲区
    // 帧结构: 7E 7E DataH DataL Valid Checksum (共6字节)
    const size_t FRAME_LEN = 6;
    bool has_valid_update = false;
    float latest_height = 0.0f;

    // 当缓冲区长度足够一帧时，循环处理
    while (m_buffer.size() >= FRAME_LEN)
    {
        // 检查帧头 0x7E 0x7E
        if (m_buffer[0] == 0x7E && m_buffer[1] == 0x7E)
        {
            // 取出关键数据
            uint8_t data_high = m_buffer[2];
            uint8_t data_low  = m_buffer[3];
            uint8_t valid_flag = m_buffer[4];
            uint8_t received_sum = m_buffer[5];

            // 计算校验和 (前5个字节相加)
            // 注意：直接在 buffer 上操作，避免拷贝
            uint16_t sum_calc = m_buffer[0] + m_buffer[1] + m_buffer[2] + m_buffer[3] + m_buffer[4];
            uint8_t calc_sum_byte = static_cast<uint8_t>(sum_calc & 0xFF);

            if (received_sum == calc_sum_byte)
            {
                // --- 校验成功 ---
                if (valid_flag == 0x80) // 检查有效位
                {
                    uint16_t raw_value = (static_cast<uint16_t>(data_high) << 8) | data_low;
                    latest_height = (static_cast<float>(raw_value) / 10.0f - 300.0f);
                    has_valid_update = true; // 标记我们拿到了数据
                }

                // 移除这完整的一帧数据，继续处理缓冲区剩余数据（可能还有更新的帧）
                m_buffer.erase(m_buffer.begin(), m_buffer.begin() + FRAME_LEN);
            }
            else
            {
                // --- 校验失败 ---
                // 虽然头是对的，但校验不对。说明这可能不是真正的头，或者数据错乱。
                // 移除第一个字节 (0x7E)，向后滑动，尝试寻找下一个头
                m_buffer.erase(m_buffer.begin());
            }
        }
        else
        {
            // --- 帧头不对 ---
            // 移除第一个字节，向后滑动寻找 0x7E
            m_buffer.erase(m_buffer.begin());
        }
    }

    // 如果解析出有效数据，更新输出
    if (has_valid_update) {
        output_height = latest_height;
        return true;
    }

    return false;
}

void SensorDataReceiver::record_radio_altitude()
{
    float new_height = 0.0f;
    
    // 调用流式解析，如果返回 true，说明 new_height 被更新为最新值
    if (readAndDecodeData(new_height))
    {
        Radio_result temp_result;
        temp_result.altitude = new_height;
        temp_result.gps_time = gps_time_str; 
        
        // 更新类成员 result
        // 在多线程环境下，如果 result 被其他线程读取，这里最好加锁
        result = temp_result;
        
        // Debug 输出可选
        // std::cout << "Updated Alt: " << new_height << std::endl;
    }
    // 如果返回 false，说明没有新数据，result 保持上一次的值，不做修改
}

void SensorDataReceiver::capture_serial_thread()
{
    std::cout << "Radio serial thread started." << std::endl;
    while(capture_thread_flag)
    {
        record_radio_altitude();
        
        // 关键：避免死循环占用 100% CPU
        // 给 CPU 一点喘息时间，同时等待串口硬件接收数据（波特率限制数据到达速度）
        // 10ms 大约对应 100Hz 的刷新率，对于雷达高度计通常足够了
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Radio serial thread stopped." << std::endl;
}

void SensorDataReceiver::start_thread()
{
    if (capture_thread_flag) return; // 防止重复启动

    capture_thread_flag = true;
    capture_thread = std::make_unique<std::thread>(&SensorDataReceiver::capture_serial_thread, this);
}

void SensorDataReceiver::stop_thread()
{
    capture_thread_flag = false;
    if (capture_thread && capture_thread->joinable())
    {
        capture_thread->join();
    }
}