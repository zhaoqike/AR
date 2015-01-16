#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include "Pattern.hpp"

#include "PatternDetector.hpp"

class ARDrawing
{
public:
	ARDrawing();
	~ARDrawing();

	void draw2dContour(PatternDetector& pd, Mat& image, PatternTrackingInfo& info, vector<Point2f> points, Mat homography, Scalar color, int lineWidth = 2);
	void drawContours(PatternDetector& pd, Mat& image, PatternTrackingInfo& info);
	void drawContours(PatternDetector& pd, Mat& image, PatternTrackingInfo& info, vector<int> indexes);
	void draw2Contours(PatternDetector& pd, Mat& image, PatternTrackingInfo& info, vector<int> matchIndexes, vector<int> estiIndexes);
};

