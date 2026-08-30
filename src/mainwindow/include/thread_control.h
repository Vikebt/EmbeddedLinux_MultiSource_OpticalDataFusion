#ifndef THREAD_CONTROL_H
#define THREAD_CONTROL_H
#include <QObject>
#include <fstream>
#include "gimbal_control.h"
class ThreadControl:public QObject
{
    Q_OBJECT
public:
    ThreadControl(std::string save_directory_, QObject *parent = nullptr);
    ~ThreadControl();
signals:
    void sig_thread_control_finished();
    void sig_update_status_bar(QString s);//更新状态栏
    void set_gimbal_config_finished();
    void sig_update_plainTextEdit_status(QString s);//更新消息框

public slots:
    //控制吊舱的姿态角 相机模式 焦距等          
    void set_gimbal_config();
    //单步调焦
    void set_gimbal_zoom(bool up);
    //焦距归一
    void set_gimbol_focus_restore();
    //一键向下
    void set_gimbol_down();
    //一键回中
    void set_gimabl_center();
    //调整角度 （已弃用）
    void set_gimbol_angle(int pitch, int yaw);
    //打开状态记录文件
    void open_file(std::string save_directory);
    //写状态文件
    void write_file(int mode);

public: 
    GimbalController* mgimbal_control;
    //吊舱控制参数
    gimbal_config mparameter;
    //控制角度的标志位
    bool control_able;
    //吊舱状态改变信息存储文件名
    std::ostringstream filename;
    std::ofstream output_file;
    std::string gps_time_str;
};

#endif // THREAD_CONTROL_H