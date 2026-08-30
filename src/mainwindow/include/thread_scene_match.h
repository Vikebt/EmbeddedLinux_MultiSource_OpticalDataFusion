/*
景像匹配线程
*/
#ifndef THREAD_SCENE_MATCH_H
#define THREAD_SCENE_MATCH_H
#include <QObject>
#include <unistd.h>
#include <iostream>
#include <random>
#include <chrono>
#include <fstream>
#include <filesystem>
#include "matching.h"
#include <QImage>


class ThreadSceneMatch:public QObject
{
    Q_OBJECT
public:
    ThreadSceneMatch(QObject *parent = nullptr);
    ~ThreadSceneMatch();
signals:
    void sig_single_match_finished(QImage img, float lon, float lat);
    void sig_match_finished();
public slots:
    void match();

public:
    bool read_cameraInfo(std::string path, struct cameraData &cd);
    bool read_imageData(std::string path,std::vector<struct imageData> &vec_id);
    bool read_microBaseImgInfo(std::string path, std::vector<struct BaseImgInfo> &vec_bd,struct briefBaseInfo &gbd);
    void Drawps(Mat &img, std::vector<cv::Point2d> srcps, int r, Scalar sc);
    void cvMat2QImage(const cv::Mat &mat, QImage& image);

};

#endif // THREAD_SCENE_MATCH_H