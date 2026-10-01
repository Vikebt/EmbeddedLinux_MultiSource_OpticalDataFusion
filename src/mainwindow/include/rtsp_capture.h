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


// 默认参数宏定义
#define DEFAULT_RTSP_URL "rtsp://wheeltec:wheeltec@192.168.1.118/554"
#define DEFAULT_SAVE_DIRECTORY "/home/wheeltec/qt_serial_ws/src/output/Photos"
#define DEFAULT_WIDTH 3840
#define DEFAULT_HEIGHT 2160

class RTSPCapture {
public:

    // 使用宏定义的默认参数构造函数
    RTSPCapture();
    ~RTSPCapture();

    // 初始化视频捕获
    bool initVideoCapture();

    // 缩放图像到指定分辨率
    void imgResize(cv::Mat& image);
    //红外图像裁剪
    void infraredProcesse(cv::Mat& image);
    
    //抓取数据线程
    void capture_frames_thread();

    //开始线程
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
    std::string gps_time_str;  //用于保存GPS时间
    
    std::atomic<bool> capture_thread_flag;//采集线程标志位
    std::unique_ptr<std::thread> capture_thread;//采集
    std::atomic<int> capture_num;//抓取图片的数量

    std::atomic<int>* share_cameramode{nullptr};
};


#endif // RTSP_CAPTURE_H
