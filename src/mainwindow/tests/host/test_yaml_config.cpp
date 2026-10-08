#include "yaml_config.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#define CHECK(expr) do { if (!(expr)) { std::cerr << "failed: " #expr << '\n'; return 1; } } while (0)

int main()
{
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path = std::filesystem::temp_directory_path() /
                      ("yaml_config_transaction_" + std::to_string(suffix) + ".yaml");
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() { std::error_code error; std::filesystem::remove(path, error); }
    } cleanup{path};

    YamlConfig config;
    const auto original_gimbal_port = config.GIMBAL_SERIAL_PORT;
    const auto original_gimbal_baud = config.GIMBAL_SERIAL_BAUDRATE;
    const auto original_radio_port = config.RADIO_SERIAL_PORT;
    {
        std::ofstream output(path);
        output << "gimbal:\n  serial_port: /dev/ttyUSB9\n  baudrate: 57600\n"
                  "radio_altimeter:\n  serial_port: invalid-relative-path\n";
    }
    bool rejected = false;
    try {
        config.readYAML(path.string());
    } catch (const std::exception&) {
        rejected = true;
    }
    CHECK(rejected);
    CHECK(config.GIMBAL_SERIAL_PORT == original_gimbal_port);
    CHECK(config.GIMBAL_SERIAL_BAUDRATE == original_gimbal_baud);
    CHECK(config.RADIO_SERIAL_PORT == original_radio_port);

    {
        std::ofstream output(path);
        output << "gimbal:\n  serial_port: /dev/ttyUSB9\n  baudrate: 57600\n";
    }
    config.readYAML(path.string());
    CHECK(config.GIMBAL_SERIAL_PORT == "/dev/ttyUSB9");
    CHECK(config.GIMBAL_SERIAL_BAUDRATE == 57600);
    CHECK(config.RADIO_SERIAL_PORT == original_radio_port);
    return 0;
}
