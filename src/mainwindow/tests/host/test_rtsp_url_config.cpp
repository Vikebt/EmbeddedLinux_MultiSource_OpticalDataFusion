#include "rtsp_url_config.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool setRtspUrl(const char* value)
{
#ifdef _WIN32
    return _putenv_s("WINDOW_CONTROL_RTSP_URL", value == nullptr ? "" : value) == 0;
#else
    return value == nullptr
        ? unsetenv("WINDOW_CONTROL_RTSP_URL") == 0
        : setenv("WINDOW_CONTROL_RTSP_URL", value, 1) == 0;
#endif
}

}  // namespace

int main()
{
    const char* original = std::getenv("WINDOW_CONTROL_RTSP_URL");
    const bool had_original = original != nullptr;
    const std::string saved = had_original ? original : "";

    const bool passed =
        setRtspUrl(nullptr) && window_control::rtspUrlFromEnvironment().empty() &&
        setRtspUrl("") && window_control::rtspUrlFromEnvironment().empty() &&
        setRtspUrl("rtsp://camera.example.invalid/live") &&
        window_control::rtspUrlFromEnvironment() == "rtsp://camera.example.invalid/live";

    const bool restored = setRtspUrl(had_original ? saved.c_str() : nullptr);
    if (!passed || !restored) {
        std::cerr << "RTSP URL must come only from explicit runtime configuration\n";
        return 1;
    }
    return 0;
}
