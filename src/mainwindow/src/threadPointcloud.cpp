#include "../include/threadPointcloud.h"
#include "threadPointcloud.h"
#include <iostream>
#include <filesystem>
#include <thread>  
#include <chrono> 
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

ThreadPointcloud::ThreadPointcloud(QObject *parent):
    thread_flag(false)
{

}

ThreadPointcloud::~ThreadPointcloud()
{

}

void ThreadPointcloud::slot_save_pointcloud()
{
    // int frame = 1;
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // while(msave_flag||(!mqueue_pts->isEmpty()))
    // {
    //     if(!mqueue_pts->isEmpty())
    //     {
    //         {
    //             QMutexLocker locker(mtx);
    //             cloud = mqueue_pts->dequeue();
    //         }
    //         save_pointcloud(frame, cloud);
    //         frame++;
    //     }
    // }
    //emit sig_save_finished();

   
    while(thread_flag)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        std::string file_name;
        bool has_pending_cloud = false;
        {
            QMutexLocker locker(mtx);
            if((!mqueue_pts->isEmpty()) && (!mqueue_names->isEmpty()))
            {
                cloud = mqueue_pts->dequeue();
                file_name = mqueue_names->dequeue();
                has_pending_cloud = true;
            }
        }
        if(has_pending_cloud)
        {
            if(cloud.points.size())
            {
                //pcl::io::savePCDFileASCII(file_name, cloud);
                pcl::io::savePLYFileBinary(file_name, cloud);
            }
            //std::cout << mqueue_pts->size() << std::endl;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

    }
    
}

void ThreadPointcloud::save_pointcloud(int frame, pcl::PointCloud<pcl::PointXYZ> cloud)
{
    (void)frame;
    (void)cloud;

}

void ThreadPointcloud::slot_save_point_data(pcl::PointCloud<pcl::PointXYZ>& point, std::string path)
{
    if(point.points.size())
    {
        pcl::io::savePCDFileASCII(path, point);
    }
}
