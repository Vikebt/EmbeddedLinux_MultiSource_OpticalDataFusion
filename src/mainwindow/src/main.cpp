#include <ros/ros.h>           // ROS核心库
#include "../include/mainwindow.h"  // 自定义的Qt主窗口类
#include <QApplication>        // Qt应用程序核心类
#include <ctime>               // 时间处理库（虽然代码中未使用）
#include <QProcess>            // Qt进程控制类（被注释掉）
#include <QFile>               // Qt文件操作类

int main(int argc, char *argv[])
{

    // system("gnome-terminal -- bash -c 'roscore'");
    // 启动一个新的 GNOME 终端并在里面执行：source 工作区环境，然后 roslaunch 启动一个 launch 文件。
    system("gnome-terminal -- bash -c 'source ~/FDILink_ROS1/devel/setup.bash && roslaunch fdi_link launch_imu.launch' "); 
 
    // 使用QProcess启动gnome-terminal
    // QProcess terminalProcess;
    // QString command = "gnome-terminal -- bash -c 'roscore'";
    // terminalProcess.start(command);
    //   // 获取终端进程的PID
    // qint64 terminalPID = terminalProcess.processId();
    // 等 3 秒（sleep）让 ROS 环境 / launch 有时间启动。
    sleep(3);
    // 创建 Qt QApplication，加载 QSS 样式表，设置本地化。
    QApplication a(argc, argv);
    QFile file("../../../src/mainwindow/resources/style.qss");
    if (file.exists()) {
        //std::cout<<"11111"<<std::endl;
        file.open(QFile::ReadOnly);
        QString styleSheet = QLatin1String(file.readAll());
        qApp->setStyleSheet(styleSheet);
        file.close();
    }

    setlocale(LC_ALL, "");

    ros::init(argc, argv, "window_control");
    MainWindow w(argc, argv);
    w.show();
   
    return a.exec();
    
}