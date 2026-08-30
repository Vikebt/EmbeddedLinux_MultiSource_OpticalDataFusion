#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

cv::Point TensorFullSearch(std::vector<cv::Mat>  rfxs, std::vector<cv::Mat> rfys, std::vector<cv::Mat> bfxs, std::vector<cv::Mat> bfys, cv::Mat realMask, cv::Mat& corr_mat);

void tofxfy(cv::Mat gimg, cv::Mat& fy, cv::Mat& fx, int w);


//cv::Mat &corr_mat
cv::Point TensorMatching(cv::Mat baseGImg, cv::Mat realGimg, cv::Mat realMask, int rs, cv::Mat& corr_mat);
