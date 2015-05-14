#pragma once


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>




using namespace std;
using namespace cv;


class Ps
{
public:
	Ps();
	~Ps();
	void boostImage(Mat& src, Mat& dst, Point center);
	void shrinkImage(Mat& src, Mat& dst, Point center);
	void radialZoomImage(Mat& src, Mat& dst, Point center);
	void rotateImage(Mat& src, Mat& dst, Point center);
};

