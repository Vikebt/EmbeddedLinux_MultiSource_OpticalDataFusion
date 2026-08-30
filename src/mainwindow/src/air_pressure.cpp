#include "../include/air_pressure.h"
#include "air_pressure.h"
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>


AirPressureSensor::AirPressureSensor(const std::string &port, int baudrate)
:port_(port), baudrate_(baudrate),capture_thread_flag(true),
headerFrame({0x01, 0x8F, 0x30, 0x30, 0x30, 0x30, 0x0D})
{
    //memset(r_buffer, 0, sizeof(r_buffer));

    mresult.altitude_a = -1;
    mresult.altitude_b = -1;
    mresult.pressure_a = -1;
    mresult.rate = -1;
    initial_height = 0;
    is_first_frame = true;
}

AirPressureSensor::~AirPressureSensor()
{
    //关闭文件流
    if (ser.isOpen()) 
    {
        ser.close();
    }
}

bool AirPressureSensor::record_air_pressure()
{
    r_buffer = std::vector<unsigned char>();
    try 
    {
        size_t available_data = ser.available();//可读取的字节数
        //std::cout<<"气压字节数"<<available_data<<std::endl;
        if (available_data > 0) 
        {
            r_buffer.resize(available_data); // 根据可用数据调整vector大小
            // 读取串口数据
            ser.read(r_buffer.data(), available_data);

            // 判断是否存在开头帧，只有在开头帧存在时才继续处理数据
            if (checkHeaderFrame(r_buffer, available_data))
            {
                //std::cout << "检测到有效的开头帧" << std::endl;
                decodeAndFormatData(available_data);
            }
            else 
            {
                std::cerr << "未检测到有效的开头帧" << std::endl;
                return false;
            }
        }
        
    } 
    catch (std::exception& e) 
    {
        std::cerr << "读取或解析串口数据时发生异常： " << std::endl;
        return false;
    }
    return true;
}

float AirPressureSensor::decodeHexToFloat(const char* hexData)
{
    const std::string mapping = "0123456789ABCDEF";
    std::string encodedStr;

    // 将hexData中的每两位进行拼接组合并转换为相应编码值
    for (int i = 0; hexData[i] != '\0' && hexData[i+1] != '\0'; i += 2) 
    {
        std::string pair(hexData + i, 2);  // 取两个字符

        if (pair == "30") encodedStr += '0';
        else if (pair == "31") encodedStr += '1';
        else if (pair == "32") encodedStr += '2';
        else if (pair == "33") encodedStr += '3';
        else if (pair == "34") encodedStr += '4';
        else if (pair == "35") encodedStr += '5';
        else if (pair == "36") encodedStr += '6';
        else if (pair == "37") encodedStr += '7';
        else if (pair == "38") encodedStr += '8';
        else if (pair == "39") encodedStr += '9';
        else if (pair == "41") encodedStr += 'A';
        else if (pair == "42") encodedStr += 'B';
        else if (pair == "43") encodedStr += 'C';
        else if (pair == "44") encodedStr += 'D';
        else if (pair == "45") encodedStr += 'E';
        else if (pair == "46") encodedStr += 'F';
    }

    // 将编码后的字符串转换为十六进制整数
    int32_t hexValue = strtol(encodedStr.c_str(), NULL, 16);

    // 将十六进制整数转换为浮点数（IEEE 754格式）
    return *(float*)&hexValue;
}

bool AirPressureSensor::checkHeaderFrame(const std::vector<unsigned char> buffer, size_t bufferSize)
{
    if (bufferSize < 7)  // 开头帧长度为 7 个字节
    {
        return false;
    }
        
    for (size_t i = 0; i <= bufferSize - 7; ++i) 
    {
        // 判断当前子数组是否与headerFrame相等
        bool isHeaderFrame = true;
        for (size_t j = 0; j < 7; ++j) 
        {
            if (buffer[i + j] != headerFrame[j]) 
            {
                isHeaderFrame = false;
                break;
            }
        }
        if (isHeaderFrame) 
        {
            return true;  // 找到匹配的开头帧
        }
    }
    return false;  // 没有找到开头帧
}

void AirPressureSensor::decodeAndFormatData(size_t available_data)
{ 
    try{
        AirPressure_result temp_result;
        temp_result.gps_time_str = gps_time_str;
        for (size_t i = 0; i < available_data - 1; i++) {//i < available_data
            // 查找帧头
            if (r_buffer[i] == 0x01) 
            {
                if (r_buffer[i + 1] == 0x82 && i + 10 < available_data) 
                { 
                    // 静压值帧
                    char hexData[15] = {0};  // 用于存储数据位的十六进制字符串
                    for (int j = 2; j <= 9; j++) {
                        sprintf(&hexData[(j - 2) * 2], "%02X", r_buffer[i + j]);//转换为16进制的字符串
                    }

                    // 解码数据
                    float pressure_a = decodeHexToFloat(hexData);
                    //result += "大气静压: " + std::to_string(pressure_a / 100.00) + " hPa  ";
                    temp_result.pressure_a = pressure_a / 100.00;
                } 
                else if (r_buffer[i + 1] == 0x87 && i + 10 < available_data) 
                {  
                    // 标准气压高度帧
                    char hexData[15] = {0};  // 用于存储数据位的十六进制字符串
                    for (int j = 2; j <= 9; j++) 
                    {
                        sprintf(&hexData[(j - 2) * 2], "%02X", r_buffer[i + j]);
                    }

                    // 解码数据
                    float altitude_a = decodeHexToFloat(hexData);
                    //result += "标准气压高度: " + std::to_string(altitude_a) + " m  ";
                    temp_result.altitude_a = altitude_a;
                }

            }
            // 查找帧头
            if (r_buffer[i] == 0x02) {
                if (r_buffer[i + 1] == 0x82 && i + 10 < available_data) 
                { // 爬升率帧
                    char hexData[15] = {0};  // 用于存储数据位的十六进制字符串
                    for (int j = 2; j <= 9; j++) 
                    {
                        sprintf(&hexData[(j - 2) * 2], "%02X", r_buffer[i + j]);
                    }

                    // 解码数据
                    float rate = decodeHexToFloat(hexData);
                    //result += "爬升率: " + std::to_string(rate) + " m/s  ";
                    temp_result.rate = rate;
                } 
                else if (r_buffer[i + 1] == 0x87 && i + 10 < available_data) 
                {  // 标准气压高度帧
                    char hexData[15] = {0};  // 用于存储数据位的十六进制字符串
                    for (int j = 2; j <= 9; j++) 
                    {
                        sprintf(&hexData[(j - 2) * 2], "%02X", r_buffer[i + j]);
                    }

                    // 解码数据
                    float altitude_b = decodeHexToFloat(hexData);
                    
                    //result += "气压校正高度: " + std::to_string(altitude_b) + " m\n";
                    
                    //相对于第一帧高度的差值
                   // temp_result.altitude_b = altitude_b - initial_height;
                    temp_result.altitude_b = altitude_b;
                    //记录上第一帧获得的气压高度 
                    if (is_first_frame)
                    {
                        is_first_frame = false;
                        initial_height = altitude_b;
                        //std::cout<<initial_height<<std::endl;
                    }
                }
            }      
        }
        mresult = temp_result;
    }
    catch (std::exception& e) 
    {
        std::cerr << "读取或解析串口数据时发生异常： " << std::endl;
    }

}
bool AirPressureSensor::initializeSerial()
{
    try 
    {
        ser.setPort(port_);
        ser.setBaudrate(baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        if (ser.isOpen()) 
        {
            std::cout << "Succeed to open air_pressure_port" << std::endl;
            return true;
        }
    } 
    catch (serial::IOException& e) 
    {
        std::cerr << "Unable to open air_pressure_port: " << e.what() << std::endl;
    } 
    return false;
    
}


void AirPressureSensor::capture_serial_thread()
{
    while(capture_thread_flag)
    {
        record_air_pressure();
    }
    std::cout<<"air串口线程停止"<<std::endl;
}

void AirPressureSensor::start_thread()
{
    if (!capture_thread) {
        capture_thread = std::make_unique<std::thread>(&AirPressureSensor::capture_serial_thread, this);
    }
}

void AirPressureSensor::stop_thread()
{
    capture_thread_flag = false;
    if (capture_thread && capture_thread->joinable()) 
    {
        capture_thread->join();
    }
}
