# 机载多源光电数据融合导航系统 · NVIDIA Jetson Orin NX

一套面向 NVIDIA Jetson Orin NX 的机载多源数据采集、融合验证和 Qt 平台管理系统。工程接入光电吊舱、RTSP 视频、SBG 组合惯导/GNSS、气压高度计、无线电高度计和点云数据，完成实时展示、记录及景象/地形匹配导航验证。

> **平台**：NVIDIA Jetson Orin NX（ARM64 / Embedded Linux）
> **技术栈**：C++17、ROS/catkin、Qt5、OpenCV、PCL、yaml-cpp、RTSP、串口、SBG INS/GNSS
> **仓库实现范围**：嵌入式 Linux 数据处理、传感器接入与记录、Qt 界面、吊舱/视频集成和融合验证代码。具体硬件部署结果以现场验证记录为准。
> **项目资料**：[验证边界与面试证据](src/mainwindow/INTERVIEW_EVIDENCE.md)
> **面试学习入口**：[五项目讲义（main）](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/tree/main/docs/interview-handbook) · [固定版本](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/tree/study-step-7-detailed-handbook/docs/interview-handbook) · [进程线程章节](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/blob/main/docs/interview-handbook/03-linux-process-thread.md) · [P4 项目故事](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/blob/main/docs/interview-handbook/06-project-stories.md#p4多源光电数据融合)

## 项目背景

机载验证需要在同一计算平台处理异构传感器数据，并保证视频、姿态、高度、GNSS 与点云记录可追溯。系统的难点不是单独读取某一个串口，而是让网络视频、串口设备、ROS 通信、Qt 界面和磁盘记录在 Orin NX 上并发运行，同时避免 UI 卡顿、线程资源泄漏和慢速存储导致的内存堆积。

## 实现概览

```text
光电吊舱 ──── 串口控制 ─┐
RTSP 视频 ─── OpenCV ───┼─> QNode 生命周期与一致性快照 ─> Qt 地面站 / 数据记录
SBG GNSS/INS ───────────┤              │
气压/无线电高度计 ──────┤              └─> 高度融合 / INS / Kalman / TERCOM / SITAN
点云数据 ───────────────┘
```

| 层级 | 模块 | 职责 |
| --- | --- | --- |
| 设备接入层 | `gimbal_control`、`rtsp_capture`、`gnss_output`、高度计、点云采集 | 串口协议、RTSP 取帧、传感器读取 |
| 数据处理层 | `Integrated_Navigation_sys`、`location` | 高度融合、INS 更新、Kalman、TERCOM/SITAN、景象匹配 |
| 集成层 | `QNode` | 设备生命周期、ROS 回调、线程协调、采集会话 |
| 应用层 | `mainwindow`、`qfi`、对话框 | Qt 状态显示、控制与验证交互 |

## 技术要点

- RTSP 视频在独立线程采集，以互斥锁保护帧快照；Qt 显示和记录流程不直接共享采集线程的 `cv::Mat`。
- RTSP 对象析构时先停止并回收工作线程；断流采用 250 ms 到 5 s 的指数退避，避免网络故障时忙循环。
- 点云落盘采用独立工作线程和最多 3 帧的有界队列；慢速磁盘场景丢弃最旧数据，保持实时链路与内存上界。
- ROS 回调只有 QNode 的 `spinOnce()` 一个调度所有者；各传感器通过加锁快照向 Qt 工作线程交付一致数据。
- 当前 QNode 通过 Qt signal 更新界面，并由 ROS subscriber 接收 GNSS/点云；代码虽保留若干 publisher 句柄，但主循环尚未调用 `publish()`，因此不把对外发布写成已完成功能。
- `config/device_profile.yaml` 集中管理设备配置，兼容旧配置；支持 `WINDOW_CONTROL_CONFIG` 注入现场参数并校验串口路径/波特率。
- CMake 使用 C++17、Release 默认、包内相对路径和 Jetson Orin NX ARMv8.2-A/Cortex-A78 工具链。

## 工程结构

```text
src/
├── mainwindow/                         主 Qt/ROS 应用包
│   ├── include/                         设备与应用接口
│   ├── src/
│   │   ├── Integrated_Navigation_sys/   融合与导航验证
│   │   ├── location/                    景象/地形匹配
│   │   └── qfi/                         飞行仪表组件
│   ├── config/                          设备档案
│   ├── launch/                          ROS 启动入口
│   └── cmake/                           Jetson Orin NX 工具链
├── sbg_ros_driver/                      SBG INS/GNSS ROS 驱动
├── serial_msgs/                         自定义 ROS 消息
└── lslidar_ls_driver/                   集成的 LiDAR ROS 驱动
```

## 构建与验证

在 Jetson Orin NX 的 Ubuntu/ROS 环境安装 Qt5、OpenCV、PCL、yaml-cpp、libudev 和相关 ROS 依赖后：

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
export WINDOW_CONTROL_CONFIG=$PWD/src/mainwindow/config/device_profile.yaml
roslaunch window_control window_control.launch
```

部署验证顺序：先校验设备配置和 `/dev` 软链接，再分别验证吊舱、RTSP、GNSS/INS、两类高度计及点云落盘，最后执行完整采集会话。`build/`、`devel/`、IDE 缓存和采集输出均不纳入版本控制。
