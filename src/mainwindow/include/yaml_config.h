#ifndef WINDOW_CONTROL_YAML_CONFIG_H
#define WINDOW_CONTROL_YAML_CONFIG_H

#include <string>
#include <yaml-cpp/yaml.h>

// Centralized device profile. Defaults keep the existing deployment operational
// when an older profile omits a newly introduced field.
class YamlConfig
{
public:
    YamlConfig();
    ~YamlConfig();

public:
    void writeYAML(const std::string& filename);
    YAML::Node readYAML(const std::string& filename);
    void printYAML(const YAML::Node&config);
    void writeYAML(const std::string& filename, YAML::Node& config);
    YAML::Node readGNSSYAML(const std::string& filename);//读取GNSS配置文件

public:
    //吊舱
    std::string GIMBAL_SERIAL_PORT;
    int GIMBAL_SERIAL_BAUDRATE;
    int PITCH_ANGLE;//俯仰角
    int YAW_ANGLE;//偏航角
    int CAMERA_MODE;//相机模式 0:可见光 1：红外
    int FOCUS;//相机焦距 

    //气压高度计
    std::string AIRPRESSURE_SERIAL_PORT;//串口
    int AIRPRESSURE_SERIAL_BAUDRATE;//波特率

    //无线电高度计
    std::string RADIO_SERIAL_PORT;//串口
    int RADIO_SERIAL_BAUDRATE;//波特率

};

#endif
