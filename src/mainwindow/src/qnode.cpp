#include "./include/qnode.h"
#include <QDebug>
#include <QDir>
#include <QDateTime>
#include <string>
#include "qnode.h"
#include <unistd.h>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <iomanip>

QNode::QNode(int argc, char** argv,QObject *parent):
    init_argc(argc),
    init_argv(argv),
    connect_succeed(false),
    save_flag(false),
    //save_flag_changed(false),
    keep_transmitting(false),
    img_num(0),
    close_flag(false),
    send_GNSS(true),
    send_radio_altimeter(true),
    send_barometric_altimeter(true),
    connect_gimbal(false),
    connect_rtsp(false),
    connect_gnss(false),
    connect_radio_altitude(false),
    connect_air_pressure(false),
    connect_point(false),
    previous_capture_num(0),
    previous_capture_num_gnss(0),
    previous_capture_num_point(0)
{
    // Prefer an explicit deployment profile, then preserve the legacy working-directory profile.
    const char* configured_path = std::getenv("WINDOW_CONTROL_CONFIG");
    QString current_path = QDir::currentPath();
    std::string yaml_file_path = configured_path != nullptr
        ? configured_path
        : (current_path.toStdString() + "/config_fly.yaml");
    try {
        config = yaml_config.readYAML(yaml_file_path);
    } catch (const std::exception& error) {
        ROS_WARN_STREAM("Cannot load device profile '" << yaml_file_path
                        << "': " << error.what() << ". Using built-in defaults.");
    }
    // create_top_folder();
    // create_folder();
    mThreadPointcloud = nullptr;

}

QNode::~QNode()
{
    
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    //std::cout<<"qqqqqq1"<<std::endl;
    if(gimbal_controller!=nullptr)
    {
        //std::cout<<"qqqqqq"<<std::endl;
        delete gimbal_controller;
        gimbal_controller = nullptr;
    }
    if(gnss_output != nullptr)
    {
        delete gnss_output;
        gnss_output = nullptr;
    }
    if(rtsp_capture != nullptr)
    {
        delete rtsp_capture;
        rtsp_capture = nullptr;
    }
    if(radio_altitude != nullptr)
    {
        delete radio_altitude;
        radio_altitude = nullptr;
    }
    if(airpressure_sensor != nullptr)
    {
        delete airpressure_sensor;
        airpressure_sensor = nullptr;
    }
    if (point_capture != nullptr)
    {
        delete point_capture;
        point_capture = nullptr;
    }
    
    if(mThreadPointcloud!=nullptr)
    {
        mThreadPointcloud->thread_flag = false;
        mThread.quit();
        mThread.wait();
        delete mThreadPointcloud;
        mThreadPointcloud = nullptr;
    }
    std::cout<<"qnode析构"<<std::endl;
}

bool QNode::init()
{
    std::cout<<"初始化"<<std::endl;

    if ( ! ros::master::check() ) {
        //连接master失败直接返回false
        return false;
    }
    //启动节点
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    //创建节点处理句柄
    ros::NodeHandle nh;

    //ros 话题初始化
    pub_radio_altitude = nh.advertise<std_msgs::String>("radioAltitudeTopic",10);
    pub_airpressure = nh.advertise<std_msgs::String>("airPressureTopic",10);
    pub_image = nh.advertise<std_msgs::String>("imageTopic",10);
    pub_gimabal_status = nh.advertise<std_msgs::String>("gimalStatusTopic",10);
    pub_GNSS = nh.advertise<std_msgs::String>("GNSSTopic",10);

    
    gimbal_controller = new GimbalController(nh);

    //配置文件的参数
    gimbal_controller->GIMBAL_SERIAL_PORT = yaml_config.GIMBAL_SERIAL_PORT;
    gimbal_controller->GIMBAL_SERIAL_BAUDRATE = yaml_config.GIMBAL_SERIAL_BAUDRATE;
    gimbal_controller->PITCH_ANGLE = yaml_config.PITCH_ANGLE;
    gimbal_controller->YAW_ANGLE = yaml_config.YAW_ANGLE;
    gimbal_controller->CAMERA_MODE = yaml_config.CAMERA_MODE;
    gimbal_controller->FOCUS = yaml_config.FOCUS;

    camera_mode = yaml_config.CAMERA_MODE;
    gimbal_controller->share_cameramode = &camera_mode;

    bool serial_flag = gimbal_controller->initializeSerial();
    if (!serial_flag)
    {
        std::cout<<"吊舱串口初始化失败"<<std::endl;
        //emit sig_update_status_bar("吊舱串口初始化失败");
    }
    else
    {
        std::cout<<"吊舱串口初始化成功"<<std::endl;
    }
    
    // 调用 initializeDevice 方法初始化设备
    bool gimal_flag = gimbal_controller->initializeDevice();
    if (!gimal_flag)
    {
        std::cout<<"吊舱初始化失败"<<std::endl;
        //emit sig_update_status_bar("吊舱初始化失败");
        emit sig_update_plainTextEdit_status("吊舱初始化失败");
    }
    else
    {
        std::cout<<"吊舱初始化成功"<<std::endl;
        emit sig_connect_succeed();
        emit sig_update_plainTextEdit_status("吊舱初始化成功");
        connect_gimbal = true;
    }


    //gimbal_controller->ser_.close();
    
    gnss_output = new GnssOutput(nh);
    /* QNode owns ROS callback dispatch through spinOnce().  A detached spinner
     * would race object destruction and process the same callback queue twice. */
    connect_gnss = true;

    point_capture = new PointCapture(nh);
    connect_point = true;

    rtsp_capture = new RTSPCapture();
    rtsp_capture->share_cameramode = &camera_mode;
    emit sig_update_status_bar("正在初始化视频流");
    if (!rtsp_capture->initVideoCapture()) {
        ROS_ERROR("视频捕获初始化失败");
        //emit sig_update_status_bar("视频捕获初始化失败");
        emit sig_update_plainTextEdit_status("视频流初始化失败");
        //return false;
    }
    else
    {
        emit sig_update_plainTextEdit_status("视频流初始化成功");
        connect_rtsp = true;
    }

    
    radio_altitude = new SensorDataReceiver(yaml_config.RADIO_SERIAL_PORT, yaml_config.RADIO_SERIAL_BAUDRATE);
    if(!radio_altitude->initializeSerial())
    {
        //emit sig_update_status_bar("无线电高度计串口初始化失败");
        emit sig_update_plainTextEdit_status("无线电高度计串口初始化失败");
    }
    else
    {
        connect_radio_altitude = true;
        emit sig_update_plainTextEdit_status("无线电高度计串口初始化成功");
    }

    airpressure_sensor = new AirPressureSensor(yaml_config.AIRPRESSURE_SERIAL_PORT, yaml_config.AIRPRESSURE_SERIAL_BAUDRATE);

    if(!airpressure_sensor->initializeSerial())
    {
        //emit sig_update_status_bar("气压高度计串口初始化失败");
        emit sig_update_plainTextEdit_status("气压高度计串口初始化失败");
    }
    else
    {
        connect_air_pressure = true;
        emit sig_update_plainTextEdit_status("气压高度计串口初始化成功");
    }
    
    if(connect_gimbal && connect_gnss && connect_rtsp && connect_air_pressure && connect_radio_altitude)
    {
        emit sig_update_status_bar("全部传感器初始化成功");
    }

    //初始化的时候就打开点云存储线程
    //保存点云线程
    mThreadPointcloud = new ThreadPointcloud();
    mThreadPointcloud->moveToThread(&mThread);
    mThreadPointcloud->mtx = &mtx;
    mThreadPointcloud->mqueue_pts = &mqueue_pts;
    mThreadPointcloud->mqueue_names = &mqueue_names;
    mThreadPointcloud->thread_flag = true;
    mThreadPointcloud->moveToThread(&mThread);
    //connect(&mThread, &QThread::finished, mThreadPointcloud, &QObject::deleteLater);
    connect(this, SIGNAL(sig_save_pts()),mThreadPointcloud, SLOT(slot_save_pointcloud()));
    //connect(this, &QNode::sig_save_pts ,mThreadPointcloud, ThreadPointcloud::slot_save_point_data);
    mThread.start();
    emit sig_save_pts();
    
    
    std::cout<<"传感器初始化完成"<<std::endl;
    emit sig_update_plainTextEdit_status("传感器初始化完成");
    connect_succeed = true;
    return true;
}

void QNode::run()
{
    if(!init())
    {
        if (gimbal_controller != nullptr && gimbal_controller->ser_.isOpen()) {
            gimbal_controller->ser_.close();
        }
        emit rosShutdown();
        emit sig_qnode_finished();
        return;
    }
    //std::cout<<"aaa"<<std::endl;
    if(connect_gimbal)
    {
       gimbal_controller->start_thread();
    }
    if(connect_air_pressure)
    {
        airpressure_sensor->start_thread();
    }
    if(connect_radio_altitude)
    {
        radio_altitude->start_thread();
    }
    if(connect_rtsp)
    {
        rtsp_capture->start_thread();
    }
    
    while (ros::ok()) 
    {
        if(keep_transmitting.load())//如果保持数据传输
        {
            // 处理ROS回调 GNSS
            ros::spinOnce();
            {
                std::lock_guard<std::mutex> lock(gps_time_mutex_);
                gps_time_str = get_current_time_string();
            }
            {
                auto start = std::chrono::steady_clock::now();
                previous_capture_num = rtsp_capture->capture_num.load();
                Gnss_result gnss_snapshot{};
                int gnss_sequence = 0;
                const bool has_gnss = gnss_output->latest(&gnss_snapshot, &gnss_sequence);
                previous_capture_num_gnss = gnss_sequence;

                if(has_gnss)
                {
                    std::lock_guard<std::mutex> lock(gps_time_mutex_);
                    gps_time_str = gnss_snapshot.gps_time;
                }

                Gimbal_result gimbal_snapshot{};
                AirPressure_result air_snapshot{};
                Radio_result radio_snapshot{};
                if (connect_gimbal && gimbal_controller->latest(&gimbal_snapshot)) {
                    gimbal_snapshot.gps_time_str = latestGpsTime();
                }
                if (connect_air_pressure) {
                    airpressure_sensor->setGpsTime(latestGpsTime());
                    (void)airpressure_sensor->latest(&air_snapshot);
                }
                if (connect_radio_altitude) {
                    radio_altitude->setGpsTime(latestGpsTime());
                    (void)radio_altitude->latest(&radio_snapshot);
                }
                
                /*-------------吊舱图片----------*/
                if(connect_rtsp)
                {
                    QImage img;
                    cv::Mat latest_frame;
                    if (rtsp_capture->latestFrame(&latest_frame)) {
                        cvMat2QImage(latest_frame, img);
                        emit sig_update_image(img);//发送信号给ui界面更新图片
                    }
                }
                
                /*----------GNSS数据------------*/
                if(send_GNSS)
                {
                    //更新GNSS信息显示
                    if (has_gnss) emit sig_update_GNSS_result(gnss_snapshot);
                }
                /*--------吊舱串口数据-----------*/
                if(connect_gimbal)
                {
                    //用解析的信息改变仪表
                    emit sig_update_gimbal_result(gimbal_snapshot);
                }
                /*---------气压高度计------------*/
                if(connect_air_pressure)
                {
                    if(send_barometric_altimeter)
                    {
                        emit sig_update_airPressure_result(air_snapshot);
                    }
                }
                if(connect_radio_altitude)
                {
                    if(send_radio_altimeter)
                    {
                        emit sig_update_radio_altitude_result(radio_snapshot);
                    }
                }
                if(save_flag.load())
                {
                    cv::Mat latest_frame;
                    if (rtsp_capture->latestFrame(&latest_frame)) {
                        pcl::PointCloud<pcl::PointXYZ> point_snapshot;
                        int point_sequence = 0;
                        if (point_capture->latest(&point_snapshot, &point_sequence) &&
                            point_sequence > previous_capture_num_point) {
                            previous_capture_num_point = point_sequence;
                        } else {
                            point_snapshot.clear();
                        }
                        save_data(latest_frame, gimbal_snapshot, gnss_snapshot,
                                  air_snapshot, radio_snapshot,
                                  point_snapshot);
                    }
                    img_num++;
                    //更新界面的保存图片数量
                    emit sig_update_label_image_num(img_num);
                }
                if(img_num != 0 && img_num % 600 == 0) //每600个 就清空一次界面 同时关闭并重新打开各个文件输出流
                {
                    emit sig_clear_message();
                }

                //不用吊舱控制采集频率 改为10HZ
                auto end = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
                //std::cout << "耗时:" << elapsed.count()  << std::endl; 
                if(elapsed < std::chrono::milliseconds(90))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(90) - elapsed);
                }


            }
            if(!connect_rtsp)
            {
                usleep(10000);
            }
        }

        if((!keep_transmitting.load())&&(!close_flag.load()))//如果是暂停了
        {
            usleep(100000);//休眠100ms
        }
       
        //界面关闭时，退出循环
        if(close_flag.load())
        {
            std::cout<<"收到关闭信号"<<std::endl;
            break;
        }
    }
    if(connect_gimbal)
    {
        //关闭串口
        gimbal_controller->capture_thread_flag = false;
        gimbal_controller->stop_thread();
        gimbal_controller->ser_.close();
        
    }
    if(connect_rtsp)
    {
        //关闭采集图片的线程
        rtsp_capture->capture_thread_flag = false;
        rtsp_capture->stop_thread();
        
    }

    if(connect_air_pressure)
    {
        airpressure_sensor->capture_thread_flag = false;
        airpressure_sensor->stop_thread();
        //airpressure_sensor->ser.close();
    }
    if(connect_radio_altitude)
    {
        //radio_altitude->capture_thread_flag = false;
        radio_altitude->stop_thread();
        //radio_altitude->ser.close();
    }
    emit rosShutdown();
    emit sig_qnode_finished();
}

void QNode::cvMat2QImage(const cv::Mat &mat, QImage& image)
{
    switch(mat.type())
    {
    case CV_8UC1:
        // QImage构造：数据，宽度，高度，每行多少字节，存储结构
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
        break;
    case CV_8UC3:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).rgbSwapped();
        //image = image.rgbSwapped(); // BRG转为RGB(颜色变成灰色)
        //Qt5.14增加了Format_BGR888
        //image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.cols * 3, QImage::Format_BGR888);
        break;
    case CV_8UC4:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        break;
    case CV_16UC4:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGBA64);
        image = image.rgbSwapped(); // BRG转为RGB
        break;
    }
}



void QNode::create_folder()
{
    // 获取当前时间
    QDateTime current_time = QDateTime::currentDateTime();
    // 将时间格式化为字符串
    std::string time_string = current_time.toString("M-d-H-m-s").toStdString();

    //std::string cutrrent_path = QDir::currentPath().toStdString();
    std::string folder_path = top_folder_path + "/" + time_string;
    photo_path = folder_path + "/Photos";
    serial_path = folder_path + "/SerialData";
    
    //创建output文件夹 以"output+时间"命名
    try {
        if (std::filesystem::create_directory(folder_path)) {
            //std::cout << "Folder created: " << folder_path << std::endl;
        } else {
            std::cout << "Folder already exists or cannot be created: " << folder_path << std::endl;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    //创建图片文件夹 以"Photos"命名
     try {
        if (std::filesystem::create_directory(photo_path)) {
            //std::cout << "Folder created: " << photo_path << std::endl;
        } else {
            std::cout << "Folder already exists or cannot be created: " << photo_path << std::endl;
        }
    } 
    catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    //创建串口信息文件夹 以"SerialData"命名
     try {
        if (std::filesystem::create_directory(serial_path)) {
            //std::cout << "Folder created: " << serial_path << std::endl;
        } else {
            std::cout << "Folder already exists or cannot be created: " << serial_path << std::endl;
        }
    } 
    catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    //std::cout<<photo_path<<std::endl;
    //std::cout<<serial_path<<std::endl;
}

void QNode::create_top_folder()//创造最上层文件夹
{
    // 获取当前时间
    QDateTime current_time = QDateTime::currentDateTime();
    // 将时间格式化为字符串
    std::string time_string = current_time.toString("M-d-H-m-s").toStdString();

    std::string cutrrent_path = QDir::currentPath().toStdString();
    top_folder_path = cutrrent_path + "/output_" + time_string;

    //创建output文件夹 以"output+时间"命名
    try {
        if (std::filesystem::create_directory(top_folder_path)) {
            //std::cout << "Folder created: " << folder_path << std::endl;
        } else {
            std::cout << "Folder already exists or cannot be created: " << top_folder_path << std::endl;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void QNode::save_data(cv::Mat &frame, Gimbal_result& gimabal_result, Gnss_result &gnss_result,
                     AirPressure_result &airPressure_result, Radio_result &radio_result,
                     pcl::PointCloud<pcl::PointXYZ>& point)
{
    if(photo_serial_path.size()<3)
    {
        return;
    }
    std::vector<std::string> aset_path = photo_serial_path;
    if(connect_gimbal)
    {
        save_gimbal_data(gimabal_result, aset_path[1]);
    }
    if(connect_rtsp)
    {
        std::stringstream filename_photo;
        filename_photo << aset_path[0] << "/photo_" << latestGpsTime() << ".jpg";
        //std::cout<<filename_photo.str()<<std::endl;
        cv::imwrite(filename_photo.str(), frame);
    }
    if(connect_gnss)
    {
        save_GNSS_data(gnss_result, aset_path[1]);
    }
    if(connect_radio_altitude)
    {
        save_radio_data(radio_result, aset_path[1]);
    }
    if(connect_air_pressure)
    {
        save_airpressure_data(airPressure_result, aset_path[1]);
    }    
    auto start = std::chrono::steady_clock::now();
    if(connect_point)
    {
        std::stringstream filename_point;
        filename_point << aset_path[2] << "/" << latestGpsTime() << ".ply";
        save_point_data(point, filename_point.str());
        // test_1117
        // std::cout << "点云name：" << filename_point.str()  << std::endl;
    }
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
    //std::cout << "点云耗时:" << elapsed.count()  << std::endl;

}


void QNode::save_gimbal_data(Gimbal_result &gimabal_result, std::string path)
{
    std::string filename = path + "/decoded_data.txt";
    // 打开文件进行写入，追加模式
    std::ofstream file_out;
    file_out.open(filename, std::ios::app);

    if (!file_out.is_open()) 
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    /***11.14.20:25 by zc***/
    // // 写入当前系统时间
    file_out << "时间: " << gimabal_result.gps_time_str << std::endl;
    /***11.14.20:25 by zc***/
    file_out << "激光测距距离: " << gimabal_result.distance << "米" << std::endl;
    file_out << "吊舱横滚角：" << gimabal_result.roll << "°" << std::endl;
    file_out << "吊舱俯仰角: " << gimabal_result.pitch<< "°" << std::endl;
    file_out << "吊舱方向角: " << gimabal_result.yaw << "°" << std::endl;
    file_out << "变焦倍数: " << gimabal_result.focus << "x" << std::endl;
    file_out << "跟踪状态: " << gimabal_result.tracking_status << std::endl;
    file_out << "目标纬度: " << gimabal_result.target_latitude<< "°" << std::endl;
    file_out << "目标经度: " << gimabal_result.target_longitude<< "°" << std::endl;
    file_out << "目标高度: " << gimabal_result.target_altitude << "米" << std::endl;
    file_out << "焦距信息(pqrs): " << gimabal_result.oss_zoom << std::endl;
    file_out << "云台工作模式: " << gimabal_result.gimbal_mode_str << std::endl;
    file_out << "计算的校验码: " << std::hex << static_cast<int>(gimabal_result.checksum) << std::endl;
    file_out << "实际的校验码: " << std::hex << static_cast<int>(gimabal_result.received_checksum) << std::endl;
    if (gimabal_result.checksum == gimabal_result.received_checksum) {
        file_out << "校验通过" << std::endl;
    } else {
        file_out << "校验未通过" << std::endl;
    }
    // 分隔符
    file_out << " " << std::endl;
    // 关闭文件
    file_out.close();
}

void QNode::save_GNSS_data(Gnss_result &gnss_result, std::string path)
{
    std::ostringstream file_name;
    file_name << path << "/gnss_output.txt";
    std::ofstream output_file;

    output_file.open(file_name.str(), std::ios::out | std::ios::app);
    if (!output_file.is_open())
    {
        ROS_ERROR("无法打开gnss_output文件进行写入");
        //throw std::runtime_error("无法打开文件进行写入");
    }
    output_file << std::fixed << std::setprecision(9) 
    << gnss_result.mode << "  "
    << gnss_result.gps_time << "  "
    << gnss_result.roll << "  " << gnss_result.pitch << "  " << gnss_result.yaw << "  " 
    << gnss_result.latitude << "  " << gnss_result.longitude << "  " << gnss_result.altitude << "  " 
    << gnss_result.speed_x << "  " << gnss_result.speed_y << "  " << gnss_result.speed_z << "  "
    << gnss_result.imu_acc_x << "  " << gnss_result.imu_acc_y << "  " << gnss_result.imu_acc_z << "  "
    << gnss_result.imu_gyro_x << "  " << gnss_result.imu_gyro_y << "  " << gnss_result.imu_gyro_z << "  "
    << "\n";
    output_file.close();

    //存时间
    std::ostringstream file_name_time;
    file_name_time << path << "/time_data.txt";
    std::ofstream output_file_time;

    output_file_time.open(file_name_time.str(), std::ios::out | std::ios::app);
    if (!output_file_time.is_open())
    {
        ROS_ERROR("无法打开time_data文件进行写入");
        //throw std::runtime_error("无法打开文件进行写入");
    }
    output_file_time << gnss_result.gps_time << "\n";
    output_file_time.close();
}

void QNode::save_airpressure_data(AirPressure_result &airPressure_result, std::string path)
{
    std::ostringstream file_name;
    file_name << path <<"/airpressure_output.txt";
    std::ofstream output_file;;
    output_file.open(file_name.str(), std::ios::app | std::ios::out);
    
    if (!output_file.is_open())
    {
        std::cout<<"无法打开airpressure文件进行写入"<<std::endl;
    }
    
    output_file << airPressure_result.gps_time_str << " " << airPressure_result.pressure_a 
                << " "<< airPressure_result.altitude_b << " " << airPressure_result.altitude_a<< std::endl;
    
    output_file.close();
}

void QNode::save_radio_data(Radio_result &radio_result, std::string path)
{
    std::ostringstream file_name;
    file_name << path <<"/radioAltitude_output.txt";
    std::ofstream output_file;;
    output_file.open(file_name.str(), std::ios::app | std::ios::out);
    
    if (!output_file.is_open())
    {
        std::cout<<"无法打开radio_result文件进行写入"<<std::endl;
    }
    output_file << radio_result.gps_time <<" " << radio_result.altitude << std::endl;
    output_file.close();
}

void QNode::save_point_data(pcl::PointCloud<pcl::PointXYZ> &point, std::string path)
{
    if(connect_point && !point.empty())
    {
        //直接在该线程中存储会导致拖慢整体进度
        // if(point.points.size())
        // {
        //     pcl::io::savePCDFileASCII(path, point);
        // }
        
        //改为开启一个线程来进行存储
        {
            QMutexLocker locker(&mtx);
            constexpr int kMaxPendingPointClouds = 3;
            if (mqueue_pts.size() >= kMaxPendingPointClouds) {
                mqueue_pts.dequeue();
                mqueue_names.dequeue();
            }
            mqueue_pts.enqueue(point);
            mqueue_names.enqueue(path);
        }
        
    }
    
}


std::string QNode::get_current_time_string()
{
    // 1. 获取当前时间点
    auto now = std::chrono::system_clock::now();

    // 2. 转换为 time_t 以获取秒级时间
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    // 3. 转换为本地时间的 tm 结构
    // 注意：localtime 不是线程安全的，在多线程环境中应使用 localtime_s 或 localtime_r
    std::tm now_tm{};
    localtime_r(&now_time_t, &now_tm);

    // 4. 获取毫秒部分
    // 获取从纪元开始到当前时间点的毫秒数
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    // 5. 格式化输出
    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%H:%M:%S")      // 格式化时:分:秒
        << '.' << std::setfill('0') << std::setw(3) << ms.count(); // 补齐3位毫秒

    return oss.str();
}

std::string QNode::latestGpsTime() const
{
    std::lock_guard<std::mutex> lock(gps_time_mutex_);
    return gps_time_str;
}
