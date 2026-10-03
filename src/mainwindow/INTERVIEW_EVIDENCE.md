# 面试证据索引：多源光电数据融合

总讲义见 [模块化五项目面试讲义](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/tree/main/docs/interview-handbook)，本项目重点对应 [C++ 与资源所有权](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/blob/main/docs/interview-handbook/01-c-cpp-memory.md)、[进程线程](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/blob/main/docs/interview-handbook/03-linux-process-thread.md)、[P4 项目故事](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/blob/main/docs/interview-handbook/06-project-stories.md#p4多源光电数据融合) 与实验手册。固定证据标签为本仓库 `study-step-2-consistent-snapshots`；总讲义固定标签为 `study-step-7-detailed-handbook`。

| 常见问题 | 代码证据 | 工程回答 |
|---|---|---|
| `detach` 有什么风险？ | `QNode::init` 与 `GnssOutput` | 原 detached spinner 可能在对象析构后继续访问；现在 ROS 回调由 QNode 单一调度，生命周期跟随 QNode |
| 数据竞争如何产生？ | 各传感器 `latest()` | 回调线程写结构体、Qt 工作线程读结构体会形成 data race；同一 mutex 下复制完整快照，避免字段来自不同帧 |
| `atomic` 能替代 mutex 吗？ | QNode 控制标志与结果快照 | 单个布尔/整数状态使用 atomic；包含 string 的复合结果必须用 mutex 保证整体一致性 |
| 四元数怎样转欧拉角？ | `GnssOutput::highFreqCallback` | IMU orientation 是四元数，必须通过 tf2 矩阵转换，不能把 x/y/z 分量直接当成 roll/pitch/yaw |
| 线程如何安全退出？ | `RTSPCapture::stop_thread`、`QNode::~QNode` | 先置停止标志，再 join/wait，最后释放设备和对象；不使用 detached 工作线程 |
| 网络断流为何要退避？ | `ReconnectBackoff` 与 RTSP 循环 | 立即重复 open 会造成 CPU/日志/网络风暴；指数退避设上限，并以 50 ms 小步等待保证停止响应 |
| 慢磁盘怎样影响实时链路？ | `QNode::save_point_data` | 点云保存使用最多 3 帧的有界队列，满时丢最旧帧，限制内存并保留较新数据 |
| 配置文件解析失败会发生什么？ | `YamlConfig::readYAML`、`test_yaml_config.cpp` | 先解析并校验候选对象，全部成功后一次性提交；坏串口路径不会留下前面字段的半更新 |
| UDP 收包为何要检查长度？ | `muti_thread.cpp::recv_thread` | 只接受完整的 28 个 double 导航报文；接收超时每 100 ms 检查停止标志，短包不再读取未初始化字段 |

## 验证边界

重连退避在 WSL Debug/Release 下各 1/1 通过。配置回滚测试在 Windows/MSVC Release + yaml-cpp DLL 下通过（连同退避 2/2）；现有第三方 DLL 与 MSVC Debug STL 不兼容，Debug 仅验证退避（1/1），不能把配置回滚写成 Debug 通过。

完整应用依赖 ROS1、Qt5、OpenCV、PCL、串口设备和 RTSP 源；当前环境不能完成 catkin 全量构建。RTSP 的 `VideoCapture` 已改为 worker join 后释放，避免跨线程 release 竞争，但底层网络读取可能阻塞，尚无固定退出时限保证；导航 UDP 改动同样仍需完整集成测试。传感器时间同步精度、RTSP 长稳或磁盘持续吞吐均无硬件验证。

宿主机可复现命令：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  tests/host/test_reconnect_backoff.cpp -Iinclude \
  -o tests/host/test_reconnect_backoff
./tests/host/test_reconnect_backoff
```
