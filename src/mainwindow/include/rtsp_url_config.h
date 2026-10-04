#ifndef WINDOW_CONTROL_RTSP_URL_CONFIG_H
#define WINDOW_CONTROL_RTSP_URL_CONFIG_H

#include <cstdlib>
#include <string>

namespace window_control {

inline std::string rtspUrlFromEnvironment()
{
    const char* value = std::getenv("WINDOW_CONTROL_RTSP_URL");
    return value == nullptr ? std::string() : std::string(value);
}

}  // namespace window_control

#endif  // WINDOW_CONTROL_RTSP_URL_CONFIG_H
