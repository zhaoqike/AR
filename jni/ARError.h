#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>

#include "PatternDetector.hpp"

using namespace std;
using namespace cv;
class ARError
{
public:
	ARError();
	~ARError();
	float point_distance(Point2f& p1, Point2f& p2);
	float computeError(PatternDetector& pd, Mat& homography);
	void printError();


	vector<float> errs;
};

