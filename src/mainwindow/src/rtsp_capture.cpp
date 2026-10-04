#include "../include/rtsp_capture.h"
#include "rtsp_capture.h"
#include "reconnect_backoff.h"
#include "rtsp_url_config.h"



// The RTSP endpoint is supplied at runtime; no credentials are compiled in.
RTSPCapture::RTSPCapture(): 
    rtsp_url_(window_control::rtspUrlFromEnvironment()),
    width_new_(DEFAULT_WIDTH), 
    height_new_(DEFAULT_HEIGHT),
    capture_thread_flag(false),
    capture_num(0)
{
}

RTSPCapture::~RTSPCapture()
{
    stop_thread();
    std::cout<<"RTSP析构"<<std::endl;
}

bool RTSPCapture::initVideoCapture() {
    if (rtsp_url_.empty()) {
        ROS_ERROR("WINDOW_CONTROL_RTSP_URL is not set; RTSP capture is disabled");
        return false;
    }
    //std::cout<<"aaaaa"<<std::endl;
     // 设置视频流分辨率 
    // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    cap_.open(rtsp_url_);
    //cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    //std::cout<<"bbbbb"<<std::endl; 
    if (!cap_.isOpened()) {
        ROS_ERROR("无法打开视频流, 请检查RTSP地址或网络连接");
        return false;
    }
   

    return true;
}


void RTSPCapture::imgResize(cv::Mat& image) {
    cv::Mat img_new;
    // if (image.cols * height_new_ >= image.rows * width_new_) {
    //     cv::resize(image, img_new, cv::Size(width_new_, image.rows * width_new_ / image.cols));
    // } else {
    //     cv::resize(image, img_new, cv::Size(image.cols * height_new_ / image.rows, height_new_));
    // }
    if (image.cols / static_cast<float>(image.rows) >= width_new_ / static_cast<float>(height_new_)) {
        cv::resize(image, img_new, cv::Size(width_new_, static_cast<int>(image.rows * width_new_ / image.cols)));
    } else {
        cv::resize(image, img_new, cv::Size(static_cast<int>(image.cols * height_new_ / image.rows), height_new_));
    }
    image = img_new;
}


void RTSPCapture::capture_frames_thread()
{
    ReconnectBackoff backoff(std::chrono::milliseconds(250),
                             std::chrono::milliseconds(5000));
    while (capture_thread_flag) 
    { 
        //抓取图片 
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            ROS_WARN("无法读取 RTSP 帧，准备退避重连");
            cap_.release();
            const auto delay = backoff.nextDelay();
            for (std::chrono::milliseconds waited(0);
                 capture_thread_flag && waited < delay;
                 waited += std::chrono::milliseconds(50)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            if (!capture_thread_flag) break;
            cap_.open(rtsp_url_);
            if (!cap_.isOpened()) {
                ROS_WARN("RTSP 重连失败，下一次延迟 %lld ms",
                         static_cast<long long>(delay.count()));
            }
            continue;
        }
        backoff.reset();
        // if(frame.type() == CV_8UC3) {
        //     // frame 是三通道的彩色图像，通常是BGR格式
        //     std::cout<<"BGR"<<std::endl;
        // } 
        // else if(frame.type() == CV_8UC1) {
        //     // frame 是单通道的灰度图像
        //     std::cout<<"gray"<<std::endl;
        // }   

        //处理图片
        //红外图片处理

        // if(camera_mode == 1)
        // {
        //     infraredProcesse(frame);
        // }
        if((share_cameramode != nullptr) && (share_cameramode->load() == 1))
        {
            infraredProcesse(frame);
        }

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_ = frame;
        }

        // 图像缩放（如果需要）
        //std::cout << "帧宽度: " << frame.cols << std::endl;
        //std::cout << "帧高度: " << frame.rows << std::endl;
        //std::cout << std::endl;

        // 在图像上添加时间戳
        capture_num++;
    }
}


void RTSPCapture::start_thread()
{
    if (!capture_thread) {
        capture_thread_flag = true;
        capture_thread = std::make_unique<std::thread>(&RTSPCapture::capture_frames_thread, this);
    }
}

void RTSPCapture::stop_thread()
{
    capture_thread_flag = false;
    if (capture_thread && capture_thread->joinable()) {
        capture_thread->join();
    }
    capture_thread.reset();
    // VideoCapture is only accessed by the worker while it is running.
    cap_.release();
}

bool RTSPCapture::latestFrame(cv::Mat* frame) const
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (current_frame_.empty()) {
        return false;
    }
    *frame = current_frame_.clone();
    return true;
}

void RTSPCapture::infraredProcesse(cv::Mat& image)
{
    if(image.cols == 1920 && image.rows == 1080)
    {
        // 指定裁剪区域，这里的参数是 (y1, y2, x1, x2)
        int y1 = 28, y2 = 1052, x1 = 320, x2 = 1600;
        cv::Mat cropped_image = image(cv::Rect(x1, y1, x2 - x1, y2 - y1));

        // 指定调整后的大小 640x512
        cv::Mat resized_image;
        cv::resize(cropped_image, resized_image, cv::Size(640, 512));
        image = resized_image;
    }
}
