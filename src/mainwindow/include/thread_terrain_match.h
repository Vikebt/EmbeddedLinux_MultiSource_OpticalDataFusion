/*
地形匹配线程
*/
#ifndef THREAD_TERRAIN_MATCH_H
#define THREAD_TERRAIN_MATCH_H
#include "data_struct.h"
#include "muti_thread.h"
#include <QObject>
#include <thread>


class ThreadTerrainMatch:public QObject
{
    Q_OBJECT
public:
    ThreadTerrainMatch(QObject *parent = nullptr);
    ~ThreadTerrainMatch();

public slots:
    void match();
    void stop_match();

signals:
    void sig_match_finished();
public:
    std::thread thread1;
    std::thread thread2;
    std::thread thread5;
    std::thread thread3;
    std::thread thread4;
};


#endif // THREAD_TERRAIN_MATCH_H