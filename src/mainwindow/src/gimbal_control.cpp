#include "../include/gimbal_control.h"
#include "gimbal_control.h"
#include <cmath>
#include <stdio.h>
//#include "gimbal_control.h"

// Constructor implementation
GimbalController::GimbalController(ros::NodeHandle& nh) : nh_(nh),
capture_thread_flag(true)
{
    photo_data = {0xFF, 0x01, 0x12, 0x00, 0x00, 0x00, 0x13};  
    start_video_data = {0xFF, 0x01, 0x12, 0x01, 0x00, 0x00, 0x14};  
    stop_video_data = {0xFF, 0x01, 0x12, 0x02, 0x00, 0x00, 0x15};  
    yaw_angle_to_zero = {0xFF, 0x01, 0x10, 0x02, 0x00, 0x00, 0x13}; 
    visible_light_mode = {0xFF, 0x01, 0x14, 0x00, 0x02, 0x00, 0x17};
    infrared_mode = {0xFF, 0x01, 0x14, 0x00, 0x03, 0x00, 0x18};
    visible_light_amplification_plus = {0xFF, 0x01, 0x00, 0x40, 0x04, 0x00, 0x45};
    visible_light_amplification_minus = {0xFF, 0x01, 0x00, 0x20, 0x04, 0x00, 0x25};
    visible_light_stops_doubling = {0xFF, 0x01, 0x00, 0x60, 0x00, 0x00, 0x61};
    //yaw_lock = {0xFF, 0x01, 0x13, 0x01, 0x00, 0x00, 0x15};
    yaw_lock = {0xFF, 0x01, 0x13, 0x00, 0x00, 0x00, 0x14};
   
    previous_focus = 1;
 

    
    // Initialize serial communication
    //initializeSerial();

    // Initialize ROS publishers
    pub_ = nh_.advertise<std_msgs::ByteMultiArray>("gps_vel", 1000);
    status_pub_ = nh_.advertise<std_msgs::Int32>("running_state", 10);
}

GimbalController::~GimbalController()
{

    if (ser_.isOpen())
    {
        ser_.close();
    }
    
    std::cout<<"gimcontrol析构"<<std::endl;
}


// Initialize serial
bool GimbalController::initializeSerial()
{
    try
    {
        ser_.setPort(GIMBAL_SERIAL_PORT); // Use predefined constants or configurations
        ser_.setBaudrate(GIMBAL_SERIAL_BAUDRATE);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(to);
        ser_.open();
    }
    catch (serial::IOException& e)
    {
        ROS_WARN("Unable to open serial port");
        return false;
    }

    if (ser_.isOpen())
    {
        ROS_INFO("Successfully opened serial port");
    }
    else
    {
        ROS_ERROR("Failed to open serial port");
        return false;
    }
    return true;
}

// Other functions remain unchanged...

// Get current time for filename
std::string GimbalController::getCurrentTimeForFilename()
{
    auto now = std::chrono::system_clock::now();
    auto now_in_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_in_time_t);

    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

// Send data and save time
void GimbalController::sendData(const std::vector<uint8_t>& data, std_msgs::Int32& status_msg)
{
    try
    {
        ser_.write(reinterpret_cast<const uint8_t*>(data.data()), data.size());
        ROS_INFO("Data sent through serial port");

        std::cout << "Time of transmission: ";
        printCurrentTime();
       

        status_msg.data = 1;
        status_pub_.publish(status_msg);
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR("Failed to send data through serial port");
    }
}

// Print current time
void GimbalController::printCurrentTime()
{
    auto now = std::chrono::system_clock::now();
    auto now_in_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_in_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

    std::tm* local_time = std::localtime(&now_in_time_t);

    std::cout << std::put_time(local_time, "%H:%M:%S")
              << "." << std::setfill('0') << std::setw(6) << now_in_microseconds.count() << std::endl;
}

// Save current time to file
void GimbalController::saveCurrentTimeToFile(const std::string& filename)
{
    std::ofstream outfile;
    outfile.open(filename, std::ios_base::app); // Append mode

    if (outfile.is_open())
    {
        auto now = std::chrono::system_clock::now();
        auto now_in_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_in_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

        std::tm* local_time = std::localtime(&now_in_time_t);
        outfile << std::put_time(local_time, "%H:%M:%S")
                << "." << std::setfill('0') << std::setw(6) << now_in_microseconds.count() 
                << std::endl;

        outfile.close();
    }
    else
    {
        ROS_ERROR("Unable to open file for logging time");
    }
}

void GimbalController::decodeData(const std::vector<uint8_t>& received_data)
{
    
    if (received_data.size() < 29) {
        std::cerr << "Data frame is too short." << std::endl;
        return;
    }
    Gimbal_result temp_result;
    temp_result.gps_time_str = "";
    // 开始解码数据帧
    // 激光测距距离，单位分米
    uint16_t distance = static_cast<uint16_t>(received_data[3]) | (static_cast<uint16_t>(received_data[4]) << 8);
    temp_result.distance = distance * 0.1;

    // 吊舱横滚角
    int16_t roll_angle = static_cast<int16_t>(received_data[5]) | (static_cast<int16_t>(received_data[6]) << 8);
    temp_result.roll = roll_angle * 0.1;

    // 吊舱俯仰角
    int16_t pitch_angle = static_cast<int16_t>(received_data[7]) | (static_cast<int16_t>(received_data[8]) << 8);
    temp_result.pitch = pitch_angle * 0.1;

    // 吊舱方向角
    int16_t yaw_angle = static_cast<int16_t>(received_data[9]) | (static_cast<int16_t>(received_data[10]) << 8);
    temp_result.yaw = yaw_angle * 0.1;

    // 变焦倍数
    int focus0 = static_cast<int>(received_data[11]);
    temp_result.focus = focus0;

    // 跟踪状态
    uint8_t tracking_status = received_data[12];
    std::string tracking_status_str = (tracking_status == 0x00) ? "未跟踪" : 
                                      (tracking_status == 0x01) ? "正在跟踪" : "状态未知";
    temp_result.tracking_status = tracking_status_str;

    // 解析目标纬度
    int32_t target_latitude = static_cast<int32_t>(received_data[13]) |
                                (static_cast<int32_t>(received_data[14]) << 8) |
                                (static_cast<int32_t>(received_data[15]) << 16) |
                                (static_cast<int32_t>(received_data[16]) << 24);
    temp_result.target_latitude = static_cast<double>(target_latitude) / 10000000;
    
    // 解析目标经度
    int32_t target_longitude = static_cast<int32_t>(received_data[17]) |
                                (static_cast<int32_t>(received_data[18]) << 8) |
                                (static_cast<int32_t>(received_data[19]) << 16) |
                                (static_cast<int32_t>(received_data[20]) << 24);
     temp_result.target_longitude = static_cast<double>(target_longitude) / 10000000;

    // 解析目标高度，单位分米
    int16_t target_altitude = static_cast<int16_t>(received_data[21]) |
                                (static_cast<int16_t>(received_data[22]) << 8);
    temp_result.target_altitude = target_altitude*0.1;

    // 解析焦距信息
    uint8_t p = received_data[23];
    uint8_t q = received_data[24];
    uint8_t r = received_data[25];
    uint8_t s = received_data[26];
    std::ostringstream oss_zoom;
    oss_zoom << std::hex << static_cast<int>(p) << static_cast<int>(q) << static_cast<int>(r) << static_cast<int>(s);
    temp_result.oss_zoom = oss_zoom.str();

    // 解析云台工作模式
    uint8_t gimbal_mode = received_data[27];
    std::string gimbal_mode_str = (gimbal_mode == 0x00) ? "跟随模式" :
                                    (gimbal_mode == 0x01) ? "锁头模式" :
                                    (gimbal_mode == 0x02) ? "一键向下" : "状态未知";
    temp_result.gimbal_mode_str = gimbal_mode_str;


    // 校验和计算
    uint8_t checksum = 0;
    for (size_t i = 0; i < 28; ++i) {
        checksum += received_data[i];
    }
    checksum &= 0xFF;  // 取低8位
    temp_result.checksum = checksum;


    // 校验和验证
    uint8_t received_checksum = received_data[28];
    temp_result.received_checksum = received_checksum;

    result = temp_result;

}

std::vector<uint8_t> GimbalController::generateAngleCommand(int angle, bool is_pitch)
{
    // Convert angle to the appropriate int16 value for the command
    int16_t value = static_cast<int16_t>(angle * 50);
    if (value < 0)
    {
        value = 65536 + value; // Adjust for 2's complement if negative
    }
    
    uint8_t data_code1 = (value >> 8) & 0xFF; // High byte
    uint8_t data_code2 = value & 0xFF;        // Low byte

    // Construct the command vector
    std::vector<uint8_t> command = {0xFF, 0x01, 0x10, static_cast<uint8_t>(is_pitch ? 0x01 : 0x02), data_code1, data_code2};
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 1; i < command.size(); ++i) {
        checksum += command[i];
    }
    checksum &= 0xFF;
    command.push_back(checksum);

    return command;
}


bool GimbalController::initializeDevice()
{
    // Define the commands to initialize the device
    std::vector<std::vector<uint8_t>> init_commands = {
        {0xFF, 0x01, 0x00, 0x60, 0x00, 0x00, 0x61},  // zoom_stop_array 可见光停止变焦
        {0xFF, 0x01, 0x11, 0x01, 0x00, 0x00, 0x13},  // stop_tracking_array 停止目标跟踪
        {0xFF, 0x01, 0x11, 0x03, 0x00, 0x00, 0x15},  // stop_detection_array 关闭目标检测识别
        {0xFF, 0x01, 0x11, 0x05, 0x00, 0x00, 0x17},  // stop_assist_tracking_array 关闭检测辅助跟踪
        {0xFF, 0x01, 0x16, 0x02, 0x00, 0x00, 0x19},  // stop_ranging_array 关闭连续测距
        {0xFF, 0x01, 0x18, 0x01, 0x00, 0x00, 0x1A},  // stop_eis_array 关闭电子稳像
        {0xFF, 0x01, 0x20, 0x00, 0x0A, 0x00, 0x2B},  // gimbal_receiver_array //设置数据回复频率为10HZ
        //{0xFF, 0x01, 0x13, 0x01, 0x00, 0x00, 0x15},  // lock_mode //切换至航向锁定
        {0xFF, 0x01, 0x13, 0x00, 0x00, 0x00, 0x14},     //航向跟随
        {0xff, 0x01, 0x12, 0x05, 0x02, 0x00, 0x1A},  // Set the resolution to 1080p 分辨率
        //{0xFF, 0x01, 0x19, 0x00, 0x00, 0x00, 0x1A},  // start_osd_array //开启画面osd信息
        {0xFF, 0x01, 0x19, 0x01, 0x00, 0x00, 0x1B},  // stop_osd_array //关闭画面osd信息
        {0xFF, 0x01, 0x14, 0x00, 0x02, 0x00, 0x17}, //默认初始化为可见光
    };

    if(CAMERA_MODE != 0)//如果是界面选择是红外
    {
        init_commands.push_back(infrared_mode);
    }
    

    // Send each command
    for (const auto& command : init_commands) {
        try {
            ser_.write(command);
            ros::Duration(0.5).sleep();
            // ROS_INFO_STREAM("Initialization command sent: " << command);
            ser_.flush();
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error sending initialization command: " << e.what());
            return false;
        }
    }

    // // Example angles to set
    // int pitch_angle = PITCH_ANGLE;//俯仰角
    // int yaw_angle = YAW_ANGLE;//偏航角

    // // Generate and send angle commands
    // std::vector<uint8_t> pitch_command = generateAngleCommand(pitch_angle, true);
    // std::vector<uint8_t> yaw_command = generateAngleCommand(yaw_angle, false);

    // try {
    //     ser_.write(pitch_command);
    //     // ROS_INFO_STREAM("Pitch angle set command: " << pitch_command);
    //     ser_.write(yaw_command);
    //     // ROS_INFO_STREAM("Yaw angle set command: " << yaw_command);
    //     ser_.flush();
    // } catch (const std::exception& e) {
    //     ROS_ERROR_STREAM("Error setting angle: " << e.what());
    //     return false;
    // }

    if(!changeAngle(PITCH_ANGLE, YAW_ANGLE))
    {
        std::cout<<"初始化吊舱角度失败!"<<std::endl;
    }
    
    //初始化的时候相机焦距为1
    int diff = FOCUS - previous_focus;
    //std::cout<<"初始化焦距差: "<<diff<<std::endl;
    if(diff > 0)//初始化的时候肯定是大于等于0的
    {
        set_gimbal_zoom(true, diff);
        previous_focus = FOCUS;
    }

    return true;
}

bool GimbalController::changeAngle(int pitch_angle, int yaw_angle)
{
    // Example angles to set
    // int pitch_angle = PITCH_ANGLE;//俯仰角
    // int yaw_angle = YAW_ANGLE;//偏航角

    // Generate and send angle commands
    std::vector<uint8_t> pitch_command = generateAngleCommand(pitch_angle, true);
    std::vector<uint8_t> yaw_command = generateAngleCommand(yaw_angle, false);

    try {
        ser_.write(pitch_command);
        ros::Duration(0.5).sleep();//LQ
        // ROS_INFO_STREAM("Pitch angle set command: " << pitch_command);
        ser_.write(yaw_command);
        ros::Duration(0.5).sleep();
        //ser_.write(yaw_lock);//LQ 航向锁定
        //ros::Duration(0.5).sleep();
        // ROS_INFO_STREAM("Yaw angle set command: " << yaw_command);
        
        ser_.flush();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error setting angle: " << e.what());
        return false;
    }

    std::vector<std::vector<uint8_t>> init_commands = {
        {0xFF, 0x01, 0x11, 0x01, 0x00, 0x00, 0x13},  // stop_tracking_array 停止目标跟踪
        {0xFF, 0x01, 0x13, 0x00, 0x00, 0x00, 0x14},     //航向跟随
    };

    for (const auto& command : init_commands) {
        try {
            ser_.write(command);
            ros::Duration(0.5).sleep();
            // ROS_INFO_STREAM("Initialization command sent: " << command);
            ser_.flush();
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error sending initialization command: " << e.what());
            return false;
        }
    }

    return true;
}

bool GimbalController::set_gimbal_config(gimbal_config para_)
{
    //设置相机模式
    if(para_.camera_mode == 0)
    {
        std_msgs::Int32 status_msg;
        // 切换到可见光模式
        sendData(visible_light_mode, status_msg);
        ROS_INFO("Switched to visible light mode.");
        ros::Duration(2).sleep();
        *share_cameramode = 0;
    }
    else if (para_.camera_mode == 1)
    {
        std_msgs::Int32 status_msg;
        // 切换到红外线模式
        sendData(infrared_mode, status_msg);
        ROS_INFO("Switched to infrared mode.");
        ros::Duration(2).sleep();
        *share_cameramode = 1;
    }
   
    //设置角度
    std::vector<uint8_t> pitch_command = generateAngleCommand(para_.pitch, true);
    std::vector<uint8_t> yaw_command = generateAngleCommand(para_.yaw, false);
    try {
        ser_.write(pitch_command);
        // ROS_INFO_STREAM("Pitch angle set command: " << pitch_command);
        ros::Duration(0.5).sleep();
        ser_.write(yaw_command);
        ros::Duration(0.5).sleep();
        // ROS_INFO_STREAM("Yaw angle set command: " << yaw_command);
        ser_.write(yaw_lock);//LQ 航向锁定
        ros::Duration(0.5).sleep();
        ser_.flush();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error setting angle: " << e.what());
        return false;
    }

    //航向跟随
    std::vector<std::vector<uint8_t>> init_commands = {
        {0xFF, 0x01, 0x11, 0x01, 0x00, 0x00, 0x13},  // stop_tracking_array 停止目标跟踪
        {0xFF, 0x01, 0x13, 0x00, 0x00, 0x00, 0x14},     //航向跟随
    };

    for (const auto& command : init_commands) {
        try {
            ser_.write(command);
            ros::Duration(0.5).sleep();
            // ROS_INFO_STREAM("Initialization command sent: " << command);
            ser_.flush();
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error sending initialization command: " << e.what());
            return false;
        }
    }


    //设置变焦倍数
    int diff = para_.focus - previous_focus;
    std::cout<<"焦距差："<<diff<<" "<<previous_focus<<std::endl;
    if(diff < 0)
    {
        set_gimbal_zoom(false, std::abs(diff));
    }
    else if(diff > 0)
    {
        set_gimbal_zoom(true, diff);
    }
    previous_focus = para_.focus;
    return true;
}

bool GimbalController::set_gimbal_zoom(bool up, int multiple)
{
    //double control_time = multiple * 0.4;//变1倍的时间约为0.4s
    std_msgs::Int32 status_msg;
    for(int i = 1; i <= multiple; i++)
    {
        if(up)
        {
            // 焦距放大
            sendData(visible_light_amplification_plus, status_msg);
            ROS_WARN("Sent zoom in command.");
            ros::Duration(0.5).sleep();
            ros::Duration(0.4).sleep();
            sendData(visible_light_stops_doubling, status_msg);
            ROS_WARN("Zoom level set to 2x.");
            ros::Duration(0.5).sleep();

        }
        else
        {
            // 焦距缩小
            sendData(visible_light_amplification_minus, status_msg);
            ROS_WARN("Sent zoom out command.");
            ros::Duration(0.5).sleep();
            ros::Duration(0.4).sleep();
            sendData(visible_light_stops_doubling, status_msg);
            ROS_WARN("Zoom level set to 2x.");
            ros::Duration(0.5).sleep();
        }
    }
    
    return true;
}

bool GimbalController::set_gimbal_down()
{

    std::vector<std::vector<uint8_t>> init_commands = {
        {0xFF, 0x01, 0x13, 0x02, 0x00, 0x00, 0x16}, //一键向下
        {0xFF, 0x01, 0x11, 0x01, 0x00, 0x00, 0x13},  // stop_tracking_array 停止目标跟踪
        {0xFF, 0x01, 0x13, 0x00, 0x00, 0x00, 0x14},  //航向跟随
    };

    for (const auto& command : init_commands) {
        try {
            ser_.write(command);
            ros::Duration(0.5).sleep();
            // ROS_INFO_STREAM("Initialization command sent: " << command);
            ser_.flush();
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error sending initialization command: " << e.what());
            return false;
        }
    }
    return true;
}

bool GimbalController::set_gimabl_center()
{
    std::vector<std::vector<uint8_t>> init_commands = {
        {0xFF, 0x01, 0x13, 0x03, 0x00, 0x00, 0x17}, //一键回中
        {0xFF, 0x01, 0x11, 0x01, 0x00, 0x00, 0x13},  // stop_tracking_array 停止目标跟踪
        {0xFF, 0x01, 0x13, 0x00, 0x00, 0x00, 0x14},  //航向跟随
    };

    for (const auto& command : init_commands) {
        try {
            ser_.write(command);
            ros::Duration(0.5).sleep();
            // ROS_INFO_STREAM("Initialization command sent: " << command);
            ser_.flush();
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error sending initialization command: " << e.what());
            return false;
        }
    }
    return true;
}

void GimbalController::capture_serial_thread()
{
    try {
    while(capture_thread_flag)
    {
        if(ser_.isOpen())
        {
            uint8_t single_data1;
            size_t bytes_read1 = ser_.read(&single_data1, 1);
            if(bytes_read1 > 0 && single_data1 == 0xEE)//找开头帧
            {
                uint8_t single_data2;
                size_t bytes_read2 = ser_.read(&single_data2, 1);
                if(bytes_read2 > 0 && single_data2 == 0x01)
                {
                    uint8_t single_data3;
                    size_t bytes_read3 = ser_.read(&single_data3, 1);
                    if(bytes_read3 > 0 && single_data3 == 0x1D)
                    {
                        std::vector<uint8_t> received_data;
                        received_data.push_back(single_data1);
                        received_data.push_back(single_data2);
                        received_data.push_back(single_data3);
                        std::vector<uint8_t> received_data_temp(26);
                        size_t bytes_read_temp = ser_.read(received_data_temp.data(), 26);
                        if(bytes_read_temp > 0)
                        {
                            received_data.insert(received_data.end(), received_data_temp.begin(), received_data_temp.end());
                            //解析串口数据
                            decodeData(received_data);
                        }
                        //std::cout<<"aaaaaa"<<std::endl;
                        // for(int p = 0; p<received_data1.size();p++)
                        // {
                        //     std::cout<<std::hex<<static_cast<int>(received_data1[p])<<" ";
                        // }
                        // std::cout<<std::endl;
                    }
                }
            }
        }
        else
        {
            break;
        }
        
    }
    }
    catch (const std::exception& e) {
        // 处理异常
        std::cerr << "Exception in thread: " << e.what() << std::endl;
    }
    std::cout<<"串口线程停止"<<std::endl;
}

void GimbalController::start_thread()
{
    if (!capture_thread) {
        capture_thread = std::make_unique<std::thread>(&GimbalController::capture_serial_thread, this);
    }
}
void GimbalController::stop_thread()
{
    capture_thread_flag = false;
    if (capture_thread && capture_thread->joinable()) 
    {
        capture_thread->join();
    }
}