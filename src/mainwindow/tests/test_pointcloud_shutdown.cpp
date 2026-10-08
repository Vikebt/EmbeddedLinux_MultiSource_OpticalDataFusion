#include "threadPointcloud.h"

#include <QMetaObject>
#include <QMutex>
#include <QQueue>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>

int main() {
    namespace fs = std::filesystem;
    const auto root = fs::temp_directory_path() /
        ("pointcloud-drain-test-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root);

    QQueue<pcl::PointCloud<pcl::PointXYZ>> clouds;
    QQueue<std::string> names;
    QMutex mutex;
    for (int index = 0; index < 2; ++index) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointXYZ point;
        point.x = static_cast<float>(index);
        point.y = 1.0F;
        point.z = 2.0F;
        cloud.push_back(point);
        clouds.enqueue(cloud);
        names.enqueue((root / ("frame-" + std::to_string(index) + ".ply")).string());
    }

    ThreadPointcloud worker;
    worker.mqueue_pts = &clouds;
    worker.mqueue_names = &names;
    worker.mtx = &mutex;
    worker.thread_flag = false;  // Shutdown requested while two saves are pending.
    const bool invoked = QMetaObject::invokeMethod(
        &worker, "slot_save_pointcloud", Qt::DirectConnection);
    const bool drained = invoked && clouds.isEmpty() && names.isEmpty() &&
        fs::exists(root / "frame-0.ply") && fs::exists(root / "frame-1.ply");
    std::error_code cleanup_error;
    fs::remove_all(root, cleanup_error);
    if (!drained) {
        std::cerr << "pending point clouds were not drained on shutdown\n";
        return 1;
    }
    return 0;
}
