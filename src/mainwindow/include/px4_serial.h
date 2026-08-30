#ifndef PX4_SERIAL_H
#define PX4_SERIAL_H

#include <iostream>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <cstring>
#include <serial/serial.h>
#include <yaml-cpp/yaml.h>

#define STX 0xFD
#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8

// 串口端口号和波特率
const std::string PX4_SERIAL_PORT="/dev/ttyUSB2";
int PX4_SERIAL_BAUDRATE = 115200;

unsigned char r_buffer[1024] = {0};
unsigned char headerFrame[] = {0xFD,0x1C,0x00,0x00,0x07,0x04,0x01,0x21,0x00,0x00,0x3C,0x6E,0x26,0x00,0x5E,0x8E,0x09,0x13,0x1D,0xF5,0xCD,0x46,0x9A,0x9C,0x00,0x00,0xEB,0xF5,0xFF,0xFF,0x02,0x00,0x0B,0x00,0x07,0x00,0xFB,0x8A,0x6E,0x04};
const uint8_t MESSAGE_CRC[256] = {50, 124, 137, 0, 237, 217, 104, 119, 117, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 14, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 189, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 208, 0, 0, 0, 0, 163, 105, 151, 35, 150, 179, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0};

struct Position {
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t hdg;
};

struct Attitude {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
};

class Px4Serial {
public:
    Px4Serial();
    void initializeSerial();         // 初始化串口
    void decodeMessage(const uint8_t* data, size_t size);
    void printCurrentTime();
    std::string getCurrentTimeForFilename();
    void saveDataToFile(const std::string& filename, const std::string& data);
    void saveCurrentTimeToFile(const std::string& filename);

    serial::Serial ser_;
    
private:
    std::string filename_;
    bool is_first_decode_ = true; // 标记是否第一次解码

    void crc_accumulate(uint8_t data, uint16_t* crcAccum);
    void crc_init(uint16_t* crcAccum);
    uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);
    void crc_accumulate_buffer(uint16_t* crcAccum, const char* pBuffer, uint16_t length);
};

#endif // PX4_SERIAL_H
