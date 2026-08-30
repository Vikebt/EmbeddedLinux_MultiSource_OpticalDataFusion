#include "yaml_config.h"
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
template <typename T>
void ReadOptional(const YAML::Node& node, const char* key, T* value)
{
    if (node[key]) {
        *value = node[key].as<T>();
    }
}

void ValidatePort(const std::string& value, const char* name)
{
    if (value.empty() || value.rfind("/dev/", 0) != 0) {
        throw std::runtime_error(std::string(name) + " must be an absolute /dev device path");
    }
}

void ValidateBaudrate(int value, const char* name)
{
    if (value <= 0) {
        throw std::runtime_error(std::string(name) + " must be positive");
    }
}
}  // namespace

YamlConfig::YamlConfig()
{
    // 串口端口号和波特率
    GIMBAL_SERIAL_PORT = "/dev/ttyCH343USB0";
    GIMBAL_SERIAL_BAUDRATE = 115200;
    PITCH_ANGLE = 0;
    YAW_ANGLE = 0;
    CAMERA_MODE = 0;
    FOCUS = 1;

    AIRPRESSURE_SERIAL_PORT = "/dev/ttyUSB1";
    AIRPRESSURE_SERIAL_BAUDRATE = 460800;

    RADIO_SERIAL_PORT = "/dev/ttyUSB3";
    RADIO_SERIAL_BAUDRATE = 230400;


}

YamlConfig::~YamlConfig()
{
}

void YamlConfig::writeYAML(const std::string &filename)
{
    YAML::Node config;
    config["gimbal"]["serial_port"] = GIMBAL_SERIAL_PORT;
    config["gimbal"]["baudrate"] = GIMBAL_SERIAL_BAUDRATE;
    config["gimbal"]["pitch_angle"] = PITCH_ANGLE;
    config["gimbal"]["yaw_angle"] = YAW_ANGLE;
    config["gimbal"]["camera_mode"] = CAMERA_MODE;
    config["gimbal"]["focus"] = FOCUS;
    config["barometric_altimeter"]["serial_port"] = AIRPRESSURE_SERIAL_PORT;
    config["barometric_altimeter"]["baudrate"] = AIRPRESSURE_SERIAL_BAUDRATE;
    config["radio_altimeter"]["serial_port"] = RADIO_SERIAL_PORT;
    config["radio_altimeter"]["baudrate"] = RADIO_SERIAL_BAUDRATE;
    YAML::Emitter emitter;
    emitter << config;
    std::ofstream fout(filename);
    fout << emitter.c_str();
    fout.close();
}

YAML::Node YamlConfig::readYAML(const std::string &filename)
{
    YAML::Node config = YAML::LoadFile(filename);
    // Accept the legacy flat profile during field upgrades, while the installed
    // profile uses grouped keys to avoid configuration scattering.
    const YAML::Node gimbal = config["gimbal"] ? config["gimbal"] : config;
    const YAML::Node barometric = config["barometric_altimeter"] ? config["barometric_altimeter"] : config;
    const YAML::Node radio = config["radio_altimeter"] ? config["radio_altimeter"] : config;

    ReadOptional(gimbal, config["gimbal"] ? "serial_port" : "GIMBAL_SERIAL_PORT", &GIMBAL_SERIAL_PORT);
    ReadOptional(gimbal, config["gimbal"] ? "baudrate" : "GIMBAL_SERIAL_BAUDRATE", &GIMBAL_SERIAL_BAUDRATE);
    ReadOptional(gimbal, config["gimbal"] ? "pitch_angle" : "PITCH_ANGLE", &PITCH_ANGLE);
    ReadOptional(gimbal, config["gimbal"] ? "yaw_angle" : "YAW_ANGLE", &YAW_ANGLE);
    ReadOptional(gimbal, config["gimbal"] ? "camera_mode" : "CAMERA_MODE", &CAMERA_MODE);
    ReadOptional(gimbal, config["gimbal"] ? "focus" : "FOCUS", &FOCUS);
    ReadOptional(barometric, config["barometric_altimeter"] ? "serial_port" : "AIRPRESSURE_SERIAL_PORT", &AIRPRESSURE_SERIAL_PORT);
    ReadOptional(barometric, config["barometric_altimeter"] ? "baudrate" : "AIRPRESSURE_SERIAL_BAUDRATE", &AIRPRESSURE_SERIAL_BAUDRATE);
    ReadOptional(radio, config["radio_altimeter"] ? "serial_port" : "RADIO_SERIAL_PORT", &RADIO_SERIAL_PORT);
    ReadOptional(radio, config["radio_altimeter"] ? "baudrate" : "RADIO_SERIAL_BAUDRATE", &RADIO_SERIAL_BAUDRATE);

    ValidatePort(GIMBAL_SERIAL_PORT, "gimbal serial_port");
    ValidatePort(AIRPRESSURE_SERIAL_PORT, "barometric_altimeter serial_port");
    ValidatePort(RADIO_SERIAL_PORT, "radio_altimeter serial_port");
    ValidateBaudrate(GIMBAL_SERIAL_BAUDRATE, "gimbal baudrate");
    ValidateBaudrate(AIRPRESSURE_SERIAL_BAUDRATE, "barometric_altimeter baudrate");
    ValidateBaudrate(RADIO_SERIAL_BAUDRATE, "radio_altimeter baudrate");

    return config;
    // return YAML::LoadFile(filename);
}

void YamlConfig::printYAML(const YAML::Node &config)
{
    (void)config;
    std::cout << "gimbal: " << GIMBAL_SERIAL_PORT << " @ " << GIMBAL_SERIAL_BAUDRATE << std::endl;
    std::cout << "barometric_altimeter: " << AIRPRESSURE_SERIAL_PORT << " @ " << AIRPRESSURE_SERIAL_BAUDRATE << std::endl;
    std::cout << "radio_altimeter: " << RADIO_SERIAL_PORT << " @ " << RADIO_SERIAL_BAUDRATE << std::endl;
}

void YamlConfig::writeYAML(const std::string &filename, YAML::Node &config)
{
    std::ofstream fout(filename);
    fout << config;
    fout.close();
}

YAML::Node YamlConfig::readGNSSYAML(const std::string& filename)
{
    YAML::Node config = YAML::LoadFile(filename);

    // // 读取driver部分
    // int frequency = config["driver"]["frequency"].as<int>();

    // // 读取odometry部分
    // bool odometryEnable = config["odometry"]["enable"].as<bool>();
    // bool publishTf = config["odometry"]["publishTf"].as<bool>();
    // std::string odomFrameId = config["odometry"]["odomFrameId"].as<std::string>();
    // std::string baseFrameId = config["odometry"]["baseFrameId"].as<std::string>();
    // std::string initFrameId = config["odometry"]["initFrameId"].as<std::string>();
    
    // // 打印读取的值
    // std::cout << "Driver Frequency: " << frequency << " Hz" << std::endl;
    // std::cout << "Odometry Enable: " << (odometryEnable ? "true" : "false") << std::endl;
    // std::cout << "Publish TF: " << (publishTf ? "true" : "false") << std::endl;
    // std::cout << "Odometry Frame ID: " << odomFrameId << std::endl;
    // std::cout << "Base Frame ID: " << baseFrameId << std::endl;
    // std::cout << "Init Frame ID: " << initFrameId << std::endl;

    return config;
}
