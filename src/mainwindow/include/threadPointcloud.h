#ifndef THREADPOINTCLOUD_H
#define THREADPOINTCLOUD_H
#include <QObject>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QQueue>
#include <QMutex>
#include <atomic>


class ThreadPointcloud: public QObject
{
    Q_OBJECT
public:
    ThreadPointcloud(QObject *parent = nullptr);
    ~ThreadPointcloud();

public:
    QQueue<pcl::PointCloud<pcl::PointXYZ>>* mqueue_pts;
    QQueue<std::string>* mqueue_names; 
    QMutex* mtx;
    std::atomic<bool> thread_flag;//存储点云标志位

private slots:
    void slot_save_pointcloud();
    void slot_save_point_data(pcl::PointCloud<pcl::PointXYZ>& point, std::string path); 
signals:
    void sig_save_finished();

private:
    void save_pointcloud(int frame, pcl::PointCloud<pcl::PointXYZ> cloud);

};

#endif // THREADPOINTCLOUD_H
