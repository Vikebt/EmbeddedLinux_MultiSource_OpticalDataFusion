#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "TensorMatch.h"
using namespace std;
using namespace cv;


cv::Point TensorFullSearch(std::vector<cv::Mat>  rfxs, std::vector<cv::Mat> rfys, std::vector<cv::Mat> bfxs, std::vector<cv::Mat> bfys, cv::Mat realMask, cv::Mat& corr_mat)
{
	static int cn = 0;

	if (rfxs.size() != rfys.size() || rfys.size() != bfxs.size() || bfxs.size() != bfys.size())
	{
		fprintf(stderr, "GPUFullSearch::rfxs, rfys, bfxs, bfys must be in the same size.");
		fflush(stdin);
		getchar();
		exit(1);
	}

	cv::Mat Xresult, Yresult, result;
	int s = rfxs.size();
	int ss = rfxs.size() * 8;

	for (int i = 0; i < ss; i++)
	{
	
		cv::matchTemplate(bfxs[i % s], rfxs[i % s], Xresult, cv::TM_CCORR, realMask);  
		cv::matchTemplate(bfys[i % s], rfys[i % s], Yresult, cv::TM_CCORR, realMask);

		cv::add(Xresult, Yresult, Xresult);
		if (i == 0)
		{
			result = Xresult.clone();
			result.setTo(0);
		}
		cv::add(Xresult, result, result);

	}

	cv::Point maxp;
	double maxv;
	cv::Mat rmask = Mat(result.rows, result.cols, CV_8U);
	rmask.setTo(0);
	rmask(Rect(2, 2, result.cols - 4, result.rows - 4)).setTo(1);
	minMaxLoc(result, NULL, &maxv, NULL, &maxp, rmask);



	corr_mat = result;
	return maxp;
}


void tofxfy(cv::Mat gimg, cv::Mat& fy, cv::Mat& fx, int w)
{

	cv::Sobel(gimg, fx, CV_32F, 1, 0, w, 1.0, 0.0, BORDER_REPLICATE);
	cv::Sobel(gimg, fy, CV_32F, 0, 1, w, 1.0, 0.0, BORDER_REPLICATE);

	cv::Mat fxy, div, mask;

	cv::multiply(fx, fy, fxy);   
	cv::pow(fx, 2, fx);         
	cv::pow(fy, 2, fy);         


	cv::add(fx, fy, div);			  
	cv::compare(div, 0, mask, CMP_EQ); 
	div.setTo(1.0f, mask);			   


	cv::subtract(fx, fy, fy);              
	cv::divide(fy, div, fy);              
	cv::divide(fxy, div, fx, 2.0);        
}



//cv::Mat &corr_mat
cv::Point TensorMatching(cv::Mat baseGImg, cv::Mat realGimg, cv::Mat realMask, int rs, cv::Mat& corr_mat)
{
	double mag = 1.0;
	cv::Mat resizeGreal, resizeGbase, realReMask, tmpReal, tmpBase;


	if (min(realGimg.cols, realGimg.rows) > rs)
	{
		mag = (double)rs / (double)min(realGimg.cols, realGimg.rows);
		int th = cvRound(realGimg.rows * mag);
		int tw = cvRound(realGimg.cols * mag);
		int bh = cvRound(baseGImg.rows * mag);
		int bw = cvRound(baseGImg.cols * mag);

		// printf("mag %.2f  th %d tw %d bh %d bw %d\n",mag,th,tw,bh,bw);


		resize(realGimg, resizeGreal, Size(tw, th), 0.0, 0.0, INTER_AREA);
		resize(baseGImg, resizeGbase, Size(bw, bh), 0.0, 0.0, INTER_AREA);
		if (!realMask.empty())
			resize(realMask, realReMask, Size(tw, th), 0.0, 0.0, INTER_NEAREST);
	}

	else
	{
		resizeGreal = realGimg.clone();
		resizeGbase = baseGImg.clone();
		realReMask = realMask.clone();
	}


	cv::GaussianBlur(resizeGreal, tmpReal, Size(3, 3), 0.7, 0, BORDER_DEFAULT);
	cv::GaussianBlur(resizeGbase, tmpBase, Size(3, 3), 0.7, 0, BORDER_DEFAULT);


	std::vector<Mat> rfys, rfxs, bfys, bfxs;
	for (int i = 0; i <= 3; i++)
	{
		cv::Mat rfx;
		cv::Mat rfy;
		cv::Mat bfx;
		cv::Mat bfy;

		double sigma = sqrt(double(1 + i * 2)) - 1.0;

		tofxfy(tmpReal, rfy, rfx, 1 + i * 2);

		tofxfy(tmpBase, bfy, bfx, 1 + i * 2);
		rfxs.push_back(rfx);
		rfys.push_back(rfy);
		bfxs.push_back(bfx);
		bfys.push_back(bfy);
	}



	Point p = TensorFullSearch(rfxs, rfys, bfxs, bfys, realReMask, corr_mat);


	p.x = cvRound(p.x / mag);
	p.y = cvRound(p.y / mag);
	return p;
}