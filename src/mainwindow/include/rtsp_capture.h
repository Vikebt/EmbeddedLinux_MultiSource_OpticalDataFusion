// rtsp_capture.h
#ifndef RTSP_CAPTURE_H
#define RTSP_CAPTURE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>  // C++17标准库
#include <thread>
#include <queue>
#include <mutex>
#include <memory>
#include <condition_variable>
#include <atomic>
#include "gnss_output.h"

#define DEFAULT_SAVE_DIRECTORY "/home/wheeltec/qt_serial_ws/src/output/Photos"
#define DEFAULT_WIDTH 3840
#define DEFAULT_HEIGHT 2160

class RTSPCapture {
public:

    RTSPCapture();
    ~RTSPCapture();

    bool initVideoCapture();

    void imgResize(cv::Mat& image);
    void infraredProcesse(cv::Mat& image);

    void capture_frames_thread();

    void start_thread();
    void stop_thread();
    bool latestFrame(cv::Mat* frame) const;

private:
    cv::VideoCapture cap_;
    std::string rtsp_url_;
    int width_new_;
    int height_new_;
    mutable std::mutex frame_mutex_;
    cv::Mat current_frame_;

public:
    std::string gps_time_str;

    std::atomic<bool> capture_thread_flag;
    std::unique_ptr<std::thread> capture_thread;
    std::atomic<int> capture_num;

    std::atomic<int>* share_cameramode{nullptr};
};

#endif // RTSP_CAPTURE_H
