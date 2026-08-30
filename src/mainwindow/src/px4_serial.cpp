#include "../include/px4_serial.h"
#include <ros/console.h>
// Constructor implementation
Px4Serial::Px4Serial(){
    // 获取当前时间作为文件名的一部分
    std::string filename = "PX4_Serial_" + getCurrentTimeForFilename() + ".txt";
    
    // Initialize serial communication
    initializeSerial();

}

std::string Px4Serial::getCurrentTimeForFilename() {
    auto now = std::chrono::system_clock::now();
    auto now_in_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_in_time_t);
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y%m%d_%H%M%S");
    return oss.str();
}

void Px4Serial::printCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto now_in_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_in_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
    std::tm* local_time = std::localtime(&now_in_time_t);
    std::cout << std::put_time(local_time, "%H:%M:%S") << "." << std::setfill('0') << std::setw(6) << now_in_microseconds.count() << std::endl;
}

void Px4Serial::saveCurrentTimeToFile(const std::string& filename) {
    auto now = std::chrono::system_clock::now();
    auto now_in_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_in_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
    std::tm* local_time = std::localtime(&now_in_time_t);
    std::ofstream file_out;
    file_out.open(filename, std::ios::app);
    if (file_out.is_open()) {
        file_out << std::put_time(local_time, "%H:%M:%S") << "." << std::setfill('0') << std::setw(6) << now_in_microseconds.count() << std::endl;
        file_out.close();
    } else {
        std::cerr << "Error opening file: " << filename << std::endl;
    }
}

void Px4Serial::saveDataToFile(const std::string& filename, const std::string& data) {
    std::ofstream file_out;
    file_out.open(filename, std::ios::app);
    if (file_out.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto now_in_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_in_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
        std::tm* local_time = std::localtime(&now_in_time_t);
        file_out << std::put_time(local_time, "%H:%M:%S") << "." << std::setfill('0') << std::setw(6) << now_in_microseconds.count() << " " << data << std::endl;
        file_out.close();
    } else {
        std::cerr << "Error opening file: " << filename << std::endl;
    }
}

void Px4Serial::decodeMessage(const uint8_t* data, size_t size) {
    unsigned short pos = 0;
    // 如果是第一次解码，设置文件名
    if (is_first_decode_) {
        filename_ = "PX4_Serial_" + getCurrentTimeForFilename() + ".txt";
        is_first_decode_ = false;  // 标记为非第一次
    }
    while (pos < size) {
        if (data[pos] == STX) {
            pos += 1;
            uint8_t LEN = data[pos];
            pos += 3;
            pos += 3;
            uint8_t MSGID = data[pos];
            pos += 3;

            uint8_t CRC_EXTRA = MESSAGE_CRC[MSGID];
            uint16_t length = pos + LEN;
            uint16_t CHECKSUM = *(uint16_t*)(data + length);

            uint8_t buff[200] = {0};
            memcpy(buff, data + pos - 9, LEN + 9);
            memcpy(buff + LEN + 9, &CRC_EXTRA, sizeof(CRC_EXTRA));

            uint16_t check = crc_calculate(buff, LEN + 10);
            if (CHECKSUM == check) {
                std::ostringstream oss;
                switch (MSGID) {
                    case 30: { // 姿态原始数据
                        Attitude attitude;
                        attitude.time_boot_ms = *(uint32_t*)(data + pos);
                        pos += 4;
                        attitude.roll = *(float*)(data + pos);
                        pos += 4;
                        attitude.pitch = *(float*)(data + pos);
                        pos += 4;
                        attitude.yaw = *(float*)(data + pos);
                        pos += 4;
                        attitude.rollspeed = *(float*)(data + pos);
                        pos += 4;
                        attitude.pitchspeed = *(float*)(data + pos);
                        pos += 4;
                        attitude.yawspeed = *(float*)(data + pos);
                        pos += 4;
                        pos += 2; // 跳过校验位

                        // 打印姿态数据
                        oss << "Attitude Data: "
                            << "Time Boot: " << attitude.time_boot_ms << " ms, "
                            << "Roll: " << attitude.roll << ", "
                            << "Pitch: " << attitude.pitch << ", "
                            << "Yaw: " << attitude.yaw << ", "
                            << "Roll Speed: " << attitude.rollspeed << ", "
                            << "Pitch Speed: " << attitude.pitchspeed << ", "
                            << "Yaw Speed: " << attitude.yawspeed;

                        ROS_INFO_STREAM(oss.str());

                        // 保存数据到文件
                        saveDataToFile(filename_, oss.str());
                        break;
                    }
                    case 33: { // 位置数据
                        Position position;
                        position.time_boot_ms = *(uint32_t*)(data + pos);
                        pos += 4;
                        position.lat = *(int32_t*)(data + pos);
                        pos += 4;
                        position.lon = *(int32_t*)(data + pos);
                        pos += 4;
                        position.alt = *(int32_t*)(data + pos);
                        pos += 4;
                        position.relative_alt = *(int32_t*)(data + pos);
                        pos += 4;
                        position.vx = *(int16_t*)(data + pos);
                        pos += 2;
                        position.vy = *(int16_t*)(data + pos);
                        pos += 2;
                        position.vz = *(int16_t*)(data + pos);
                        pos += 2;
                        position.hdg = *(uint16_t*)(data + pos);
                        pos += 2;

                        // 打印位置数据
                        oss << "Position Data: "
                            << "Time Boot: " << position.time_boot_ms << " ms, "
                            << "Latitude: " << position.lat << ", "
                            << "Longitude: " << position.lon << ", "
                            << "Altitude: " << position.alt << ", "
                            << "Relative Altitude: " << position.relative_alt << ", "
                            << "Vx: " << position.vx << ", "
                            << "Vy: " << position.vy << ", "
                            << "Vz: " << position.vz << ", "
                            << "Heading: " << position.hdg;

                        ROS_INFO_STREAM(oss.str());

                        // 保存数据到文件
                        saveDataToFile(filename_, oss.str());
                        break;
                    }
                    default:
                        pos += LEN + 2;
                        break;
                }
            } else {
                pos += LEN + 2;
            }
        } else {
            pos++;
        }
    }
}

void Px4Serial::initializeSerial() {
    try {
        ser_.setPort(PX4_SERIAL_PORT);
        ser_.setBaudrate(PX4_SERIAL_BAUDRATE);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(to);
        ser_.open();
        if (ser_.isOpen()) {
            ROS_INFO_STREAM("Serial Port initialized");
        } else {
            ROS_ERROR_STREAM("Unable to open port " << PX4_SERIAL_PORT);
        }
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port " << PX4_SERIAL_PORT);
    }
}

void Px4Serial::crc_accumulate(uint8_t data, uint16_t* crcAccum) {
    uint8_t tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

void Px4Serial::crc_init(uint16_t* crcAccum) {
    *crcAccum = X25_INIT_CRC;
}

uint16_t Px4Serial::crc_calculate(const uint8_t* pBuffer, uint16_t length) {
    uint16_t crcTmp;
    crc_init(&crcTmp);
    while (length--) {
        crc_accumulate(*pBuffer++, &crcTmp);
    }
    return crcTmp;
}

void Px4Serial::crc_accumulate_buffer(uint16_t* crcAccum, const char* pBuffer, uint16_t length) {
    const uint8_t* p = (const uint8_t*)pBuffer;
    while (length--) {
        crc_accumulate(*p++, crcAccum);
    }
}
