
#ifndef EXAMPLE_MARKERLESS_AR_PATTERN_HPP
#define EXAMPLE_MARKERLESS_AR_PATTERN_HPP

////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"
#include "CameraCalibration.hpp"

#include <opencv2/opencv.hpp>
using namespace cv;

struct Eye
{
	Eye(float _x,float _y,float _z);
	
	void computeDistance();
	friend ostream& operator<<(ostream& os, Eye& eye);
	
	
	float x, y, z, distance;
};

struct KeyFrame
{
	Size                  size;
	Mat                   frame;
	Mat                   grayImg;
	Point3f 			center;

	vector<KeyPoint> keypoints;
	Mat                   descriptors;
	Rect rect;
	vector<Point2f>  points2d;
	vector<Point3f>  points3d;
};

struct Layer
{
	int level;
	vector<KeyFrame> keyframeList;
};

struct IndexInPattern
{
	int layerIndex;
	int kfIndexInLayer;

	IndexInPattern(int layerIdx, int kfIdxInLayer)
	{
		layerIndex = layerIdx;
		kfIndexInLayer = kfIdxInLayer;
	}
};
/**
 * Store the image data and computed descriptors of target pattern
 */
struct Pattern
{
	Size                  size;
	Mat                   frame;
	Mat                   grayImg;

	vector<KeyPoint> keypoints;
	Mat                   descriptors;

	vector<Point2f>  points2d;
	vector<Point3f>  points3d;

	vector<KeyFrame> keyframeList;
	vector<Layer> layerList;
	int layerIndex;
	int kfIndexInLayer;
	
	
	
	
	int keyframeIndex;

};

/**
 * Intermediate pattern tracking info structure
 */
struct PatternTrackingInfo
{
	Mat                   homography;
	vector<Point2f>  points2d;
	Transformation            pose3d;


	void draw2dContour(Mat& image, Scalar color, int lineWidth=2) const;

	/**
	 * Compute pattern pose using PnP algorithm
	 */
	void computePose(const Pattern& pattern, const CameraCalibration& calibration);
};

#endif
