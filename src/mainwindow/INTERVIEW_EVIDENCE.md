# 面试证据索引：多源光电数据融合

| 常见问题 | 代码证据 | 工程回答 |
|---|---|---|
| `detach` 有什么风险？ | `QNode::init` 与 `GnssOutput` | 原 detached spinner 可能在对象析构后继续访问；现在 ROS 回调由 QNode 单一调度，生命周期跟随 QNode |
| 数据竞争如何产生？ | 各传感器 `latest()` | 回调线程写结构体、Qt 工作线程读结构体会形成 data race；同一 mutex 下复制完整快照，避免字段来自不同帧 |
| `atomic` 能替代 mutex 吗？ | QNode 控制标志与结果快照 | 单个布尔/整数状态使用 atomic；包含 string 的复合结果必须用 mutex 保证整体一致性 |
| 四元数怎样转欧拉角？ | `GnssOutput::highFreqCallback` | IMU orientation 是四元数，必须通过 tf2 矩阵转换，不能把 x/y/z 分量直接当成 roll/pitch/yaw |
| 线程如何安全退出？ | `RTSPCapture::stop_thread`、`QNode::~QNode` | 先置停止标志，再 join/wait，最后释放设备和对象；不使用 detached 工作线程 |
| 网络断流为何要退避？ | `ReconnectBackoff` 与 RTSP 循环 | 立即重复 open 会造成 CPU/日志/网络风暴；指数退避设上限，并以 50 ms 小步等待保证停止响应 |
| 慢磁盘怎样影响实时链路？ | `QNode::save_point_data` | 点云保存使用最多 3 帧的有界队列，满时丢最旧帧，限制内存并保留较新数据 |

## 验证边界

主机测试覆盖重连退避序列（250、500、1000、2000、4000、5000 ms，上限保持 5000 ms，成功后复位）。本次还执行了 `git diff --check` 和并发反模式检索；两项均通过。

完整应用依赖 ROS1、Qt5、OpenCV、PCL、串口设备和 RTSP 源，当前 Windows 主机不具备这套目标环境，因此这里只能确认纯 C++ 策略测试与源码一致性，不能声称完成 catkin 全量构建，更不能声称传感器时间同步精度、RTSP 长稳或磁盘持续吞吐已经通过硬件验证。

宿主机可复现命令：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  tests/host/test_reconnect_backoff.cpp -Iinclude \
  -o tests/host/test_reconnect_backoff
./tests/host/test_reconnect_backoff
```
