#include "thread_control.h"
#include <chrono>
#include <ctime>

ThreadControl::ThreadControl(std::string save_directory_, QObject *parent):
control_able(true)
{
    open_file(save_directory_);
}

ThreadControl::~ThreadControl()
{

}

void ThreadControl::set_gimbal_config()
{
    if(mgimbal_control->set_gimbal_config(mparameter))
    {
        //emit sig_update_status_bar("吊舱参数设置成功");
        emit sig_update_plainTextEdit_status("吊舱参数设置成功");
        //////////////////////////记录状态///////////////////////////////
        write_file(0);
        ///////////////////////////////////////////////////////////////
    }
    else
    {
        //emit sig_update_status_bar("吊舱参数设置失败");
        emit sig_update_plainTextEdit_status("吊舱参数设置失败");
    }
    emit set_gimbal_config_finished();
    control_able = true;//手动调整完成之后 才允许定时调整角度
}

void ThreadControl::set_gimbal_zoom(bool up)
{ 
    mgimbal_control->set_gimbal_zoom(up,1);
    if(up)
    {
        //更新当前的焦距
        if(mgimbal_control->previous_focus < 8)
        {
            mgimbal_control->previous_focus++;
        }
    }
    else
    {
        //更新当前的焦距
        if(mgimbal_control->previous_focus > 1)
        {
            mgimbal_control->previous_focus--;
        }
    }

    //////////////////////////记录状态///////////////////////////////
    write_file(1);
    ///////////////////////////////////////////////////////////////

    emit set_gimbal_config_finished();
    control_able = true;//手动调整完成之后 才允许定时调整角度
}

void ThreadControl::set_gimbol_focus_restore()
{
    if(mgimbal_control->previous_focus != 1)
    {
        mgimbal_control->set_gimbal_zoom(false, mgimbal_control->previous_focus - 1);
    }
}

void ThreadControl::set_gimbol_down()
{
    if(mgimbal_control->set_gimbal_down())
    {
        //////////////////////////记录状态///////////////////////////////
        write_file(2);
        ///////////////////////////////////////////////////////////////
        emit sig_update_plainTextEdit_status("吊舱一键向下成功");
    }
    else
    {
        emit sig_update_plainTextEdit_status("吊舱一键向下失败");
    }
    emit set_gimbal_config_finished();
}

void ThreadControl::set_gimabl_center()
{
    if(mgimbal_control->set_gimabl_center())
    {
        //////////////////////////记录状态///////////////////////////////
        write_file(3);
        ///////////////////////////////////////////////////////////////
        emit sig_update_plainTextEdit_status("吊舱一键回中成功");
    }
    else
    {
        emit sig_update_plainTextEdit_status("吊舱一键回中失败");
    }
    emit set_gimbal_config_finished();
}

void ThreadControl::set_gimbol_angle(int pitch, int yaw)
{
    if(control_able)
    {
        mgimbal_control->changeAngle(pitch, yaw);
    }
}

void ThreadControl::open_file(std::string save_directory)
{
    // 获取当前系统时间并生成文件名
    auto now = std::chrono::system_clock::now();
    auto now_in_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_in_time_t);
    
    //filename << "/home/wheeltec/qt_serial_ws/src/output/SerialData/status_record_" << std::put_time(local_time, "%Y-%m-%d_%H-%M-%S") << ".txt";
    filename << save_directory << "/status_record_"<<std::put_time(local_time, "%Y-%m-%d_%H-%M-%S") << ".txt";
    output_file.open(filename.str(), std::ios::app);
    if (!output_file.is_open())
    {
        std::cout<<"无法打开状态记录文件进行写入"<<std::endl;   
    }
    output_file.close();
}

void ThreadControl::write_file(int mode)
{
    output_file.open(filename.str(), std::ios::app);
    if (output_file.is_open())
    {
        if(mode == 0) //对应的是用设置按钮来调整吊舱
        {
            output_file << "时间: "<< gps_time_str << std::endl;
            if(mparameter.camera_mode == 0)
            {
                output_file << "模式: 可见光"<<std::endl;
            }
            else if(mparameter.camera_mode == 1)
            {
                output_file << "模式: 红外"<<std::endl;
            }
            output_file << "俯仰角: "<<mparameter.pitch << " 偏航角: "<<mparameter.yaw<<" 滚转角: "<< mparameter.roll << std::endl;
            output_file << "变焦倍数: "<<mparameter.focus << "x"<<std::endl;
            output_file << std::endl;
        }
        else if(mode == 1)  //对应的单步调焦
        {
            output_file << "时间: "<< gps_time_str << std::endl;
            output_file << "调整焦距: "<<mgimbal_control->previous_focus << "x"<<std::endl;
            output_file << std::endl;
        }
        else if(mode == 2)  //对应的一键朝下
        {
            output_file << "时间: "<< gps_time_str << std::endl;
            output_file << "一键向下"<<std::endl;
            output_file << std::endl;
        }
        else if(mode == 3)  //对应的一键回中
        {
            output_file << "时间: "<< gps_time_str << std::endl;
            output_file << "一键回中"<<std::endl;
            output_file << std::endl;
        }
    }
    output_file.close();
}


