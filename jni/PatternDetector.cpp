

////////////////////////////////////////////////////////////////////
// File includes:
#include "PatternDetector.hpp"
#include "DebugHelpers.hpp"

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <cmath>
//#include <iterator>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <sstream>
#include "Timer.h"

#include "Globals.h"




#ifndef WIN32
#include <unistd.h>
#include <jni.h>
#include <android/log.h>
#define  LOG_TAG    "libgljni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

#endif




extern bool isMultiScale;


extern bool isShowRects;
extern bool isShowPoints;



int StringToInt(string str)
{
	int result = 0;
	for (int i = 0; i < str.length(); i++)
	{
		result *= 10;
		result += str[i] - '0';
	}
	return result;
}


string intToString4(int i)
{
	string result = "0000";
	int index = result.length() - 1;
	while (i)
	{
		result[index] += i % 10;
		i /= 10;
		index--;
	}
	return result;
}

string intToString(int i)
{
	stringstream ss;
	ss << i;
	return ss.str();
}

void printMat(string name, Mat& mat)
{
	cout << name << endl;
	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			cout << mat.at<double>(i, j) << ' ';
		}
		cout << endl;
	}
	cout << endl;
}

PatternDetector::PatternDetector(Ptr<FeatureDetector> detector,
	Ptr<DescriptorExtractor> extractor,
	Ptr<DescriptorMatcher> matcher,
	bool ratioTest,
	bool wrap,
	bool opticalFlow,
	bool estimatedHomoFound)
	: m_detector(detector)
	, m_extractor(extractor)
	, m_matcher(matcher)
	, enableRatioTest(ratioTest)
	, enableWrap(wrap)
	, enableOpticalFlow(opticalFlow)
	, estimatedHomographyFound(estimatedHomoFound)
	, homographyReprojectionThreshold(5)
	, m_lostFrameNum(0)
	, m_opticalFrameNum(0)
{
	errs.reserve(10000);
	screenPolygon = makeScreenPoly();
}


void PatternDetector::train(const Pattern& pattern)
{
	// Store the pattern object
	m_pattern = pattern;

	// API of DescriptorMatcher is somewhat tricky
	// First we clear old train data:
	m_matcher->clear();

	// Then we add vector of descriptors (each descriptors matrix describe one image).
	// This allows us to perform search across multiple images:
	vector<Mat> descriptors(1);

	/*if(pattern.descriptors.type()!=CV_32F) {
		pattern.descriptors.convertTo(pattern.descriptors, CV_32F);
		}*/


	descriptors[0] = pattern.descriptors.clone();








	m_matcher->add(descriptors);

	// After adding train data perform actual train:
	m_matcher->train();
}

void PatternDetector::trainPatternList(const vector<Pattern>& patternList)
{

}

int PatternDetector::findKeyframe()
{
	return 0;
}
void PatternDetector::makeKeyFrame(const Mat& rangeImage, Rect& range, Size& oriSize, KeyFrame& keyframe)
{


	//KeyFrame pattern;

	int numImages = 4;
	float step = sqrtf(2.0f);

	// Store original image in pattern structure
	keyframe.size = Size(rangeImage.cols, rangeImage.rows);
	keyframe.frame = rangeImage.clone();
	keyframe.rect = range;
	getGray(rangeImage, keyframe.grayImg);

	// Build 2d and 3d contours (3d contour lie in XY plane since it's planar)
	keyframe.points2d.resize(4);
	keyframe.points3d.resize(4);

	// Image dimensions
	const float w = oriSize.width;
	const float h = oriSize.height;

	// Normalized dimensions:
	const float maxSize = (max)(w, h);
	const float unitW = w / maxSize;
	const float unitH = h / maxSize;
	int ltx = range.x;
	int lty = range.y;
	int rtx = range.x + range.width;
	int rty = range.y;
	int lbx = range.x;
	int lby = range.y + range.height;
	int rbx = range.x + range.width;
	int rby = range.y + range.height;

	keyframe.points2d[0] = Point2f(ltx, lty);
	keyframe.points2d[1] = Point2f(lbx, lby);
	keyframe.points2d[2] = Point2f(rbx, rby);
	keyframe.points2d[3] = Point2f(rtx, rty);

	float ltw = -unitW + 2 * unitW*ltx / w;
	float lth = -unitH + 2 * unitH*lty / h;

	float rtw = -unitW + 2 * unitW*rtx / w;
	float rth = -unitH + 2 * unitH*rty / h;

	float lbw = -unitW + 2 * unitW*lbx / w;
	float lbh = -unitH + 2 * unitH*lby / h;

	float rbw = -unitW + 2 * unitW*rbx / w;
	float rbh = -unitH + 2 * unitH*rby / h;

	keyframe.points3d[0] = Point3f(-unitW, -unitH, 0);
	keyframe.points3d[1] = Point3f(unitW, -unitH, 0);
	keyframe.points3d[2] = Point3f(unitW, unitH, 0);
	keyframe.points3d[3] = Point3f(-unitW, unitH, 0);

	keyframe.center.x = (ltw + rtw) / 2;
	keyframe.center.y = (lth + lbh) / 2;
	keyframe.center.z = 0;

	double scalew = (double)screenWidth / (double)keyframe.frame.cols;
	double scaleh = (double)screenHeight / (double)keyframe.frame.rows;
	//double scale=(scalew+scaleh)/2;
	double scale = getWindowDivPictureScale(keyframe.frame.cols, keyframe.frame.rows);

	//double scale = 1.0;

	cout << "scalew scaleh scale: " << scalew << "  " << scaleh << "  " << scale << endl;


	Size dstsize(keyframe.grayImg.cols*scale, keyframe.grayImg.rows*scale);
	resize(keyframe.grayImg, keyframe.grayImg, dstsize);

	extractFeatures(keyframe.grayImg, keyframe.keypoints, keyframe.descriptors);

	for (int i = 0; i < keyframe.keypoints.size(); i++)
	{
		keyframe.keypoints[i].pt.x /= scale;
		keyframe.keypoints[i].pt.y /= scale;
		keyframe.keypoints[i].pt.x += range.x;
		keyframe.keypoints[i].pt.y += range.y;
	}
}

double PatternDetector::getWindowDivPictureScale(int width, int height)
{
	double scalew = (double)screenWidth / (double)width;
	double scaleh = (double)screenHeight / (double)height;
	double scale = (scalew + scaleh) / 2;
	return scale;
}

double PatternDetector::getPictureDivWindowScale(int width, int height)
{
	double scalew = (double)width / (double)screenWidth;
	double scaleh = (double)height / (double)screenHeight;
	double scale = (scalew + scaleh) / 2;
	return scale;
}

int PatternDetector::getLayerNum(int width, int height)
{
	if (isMultiScale)
	{

		double scale = getPictureDivWindowScale(width, height);
		if (false){
			int level = log(scale) / log(2);
			return 2 * level;
		}
		else{
			int level = 1;
			double dlevel = log(scale) / log(2);
			if (dlevel < 1)
			{
				level = 1;
			}
			else
			{
				level = ceil(dlevel);
			}
			return (std::min)(2 * level, 4);
		}
	}
	else
	{
		return 1;
	}

}

void PatternDetector::makeKeyFrame(const Mat& oriimage, Rect& range, KeyFrame& kf)
{
	Size oriSize = oriimage.size();
	Mat rectimage;
	oriimage(range).copyTo(rectimage);

	makeKeyFrame(rectimage, range, oriSize, kf);
}

void PatternDetector::cutImage(Mat& image, int level, vector<Rect>& rectList)
{
	rectList.clear();
	Size oriSize = image.size();
	for (int k = 0; k <= level; k++)
	{
		int num = pow(2, k);
		for (int i = 0; i < num; i++)
		{
			for (int j = 0; j < num; j++)
			{
				int w = oriSize.width / num;
				int h = oriSize.height / num;
				int left = i*w;
				int top = j*w;

				Rect r(left, top, w, h);
				rectList.push_back(r);
			}
		}
	}
}

double PatternDetector::calcScaleFromLevel(int level)
{
	double scale = 1;
	if (level % 2) //odd
	{
		level /= 2;
		scale = pow(2, level);
		scale *= 1.5;
	}
	else //even
	{
		level /= 2;
		scale = pow(2, level);
	}
	return scale;
}

vector<Rect> PatternDetector::getRectOfLayer(const Mat& img, int level)
{
	vector<Rect> rectList;
	rectList.clear();
	double scale = calcScaleFromLevel(level);

	Size oriSize = img.size();
	int oriWidth = oriSize.width;
	int oriHeight = oriSize.height;

	int width = (double)oriSize.width / scale;
	int height = (double)oriSize.height / scale;
	Size size(width, height);
	int num = ceil(scale);
	int startx = 0;
	int endx = oriWidth - width;
	int starty = 0;
	int endy = oriHeight - height;

	cout << "start end" << endl;
	//cout << startx << "  " << endx << "  " << starty << "  " << endy << endl;
	cout << "scale and num: " << scale << "  " << num << endl;
	if (num == 1)
	{
		//KeyFrame kf;
		Rect rect = Rect(0, 0, oriWidth, oriHeight);
		//layer.keyframeList.push_back(kf);
		rectList.push_back(rect);
	}
	else
	{
		vector<int> xList;
		vector<int> yList;
		double xRange = (double)(endx - startx) / (double)(num - 1);
		double yRange = (double)(endy - starty) / (double)(num - 1);
		cout << "range" << endl;
		cout << xRange << "  " << yRange << endl;
		for (int i = 0; i < num; i++)
		{
			xList.push_back(i*xRange);
			yList.push_back(i*yRange);
		}
		for (int i = 0; i < num; i++)
		{
			for (int j = 0; j < num; j++)
			{
				Rect rect = Rect(xList[i], yList[j], width, height);

				if (rect.x < 0)
				{
					//cout << "layer: " << level << "  " << i << "  " << j << " xleft" << endl;
					//cout << kf.rect.x << endl;
					rect.x = 0;
				}
				if (rect.y < 0)
				{
					//cout << "layer: " << level << "  " << i << "  " << j << " ytop" << endl;
					//cout << kf.rect.y << endl;
					rect.y = 0;
				}
				if (rect.x + rect.width>oriWidth)
				{
					//cout << "layer: " << level << "  " << i << "  " << j << " xright" << endl;
					//cout << kf.rect.x + kf.rect.width << "  " << oriWidth << endl;
					rect.x -= rect.x + rect.width - oriWidth;
				}
				if (rect.y + rect.height > oriHeight)
				{
					//cout << "layer: " << level << "  " << i << "  " << j << " ybottom" << endl;
					//cout << kf.rect.y + kf.rect.height << "  " << oriHeight << endl;
					rect.y -= rect.y + rect.height - oriHeight;
				}
				rectList.push_back(rect);
			}
		}
	}
	return rectList;
}

void PatternDetector::makeLayer(const Mat& img, Layer& layer, int lev)
{
	vector<Rect> rectList = getRectOfLayer(img, lev);
	layer.keyframeList.resize(rectList.size());
	for (int i = 0; i < rectList.size(); i++)
	{
		makeKeyFrame(img, rectList[i], layer.keyframeList[i]);
	}
}

void PatternDetector::makeKeyFrameList(const Mat& img, Layer& layer, int lev)
{
	cout << "=====================================" << endl;
	int level = lev;
	cout << "layer: " << lev << " start" << endl;
	Size oriSize = img.size();
	int oriWidth = oriSize.width;
	int oriHeight = oriSize.height;
	/*double scale = 1;
	if (level % 2) //odd
	{
	level /= 2;
	scale = pow(2, level);
	scale *= 1.5;
	}
	else //even
	{
	level /= 2;
	scale = pow(2, level);
	}*/


	double scale = calcScaleFromLevel(lev);
	int width = (double)oriSize.width / scale;
	int height = (double)oriSize.height / scale;
	Size size(width, height);
	int num = ceil(scale);
	int startx = 0;
	int endx = oriWidth - width;
	int starty = 0;
	int endy = oriHeight - height;

	cout << "start end" << endl;
	//cout << startx << "  " << endx << "  " << starty << "  " << endy << endl;
	cout << "scale and num: " << scale << "  " << num << endl;
	if (num == 1)
	{
		KeyFrame kf;
		kf.rect = Rect(0, 0, oriWidth, oriHeight);
		layer.keyframeList.push_back(kf);
	}
	else
	{
		vector<int> xList;
		vector<int> yList;
		double xRange = (double)(endx - startx) / (double)(num - 1);
		double yRange = (double)(endy - starty) / (double)(num - 1);
		cout << "range" << endl;
		cout << xRange << "  " << yRange << endl;
		for (int i = 0; i < num; i++)
		{
			xList.push_back(i*xRange);
			yList.push_back(i*yRange);
		}
		/*for (int i = 0; i < num; i++)
		{
		cout << "list " << i << endl;
		cout << xList[i] << "  " << yList[i] << endl;
		}*/
		for (int i = 0; i < num; i++)
		{
			for (int j = 0; j < num; j++)
			{
				KeyFrame kf;
				kf.rect = Rect(xList[i], yList[j], width, height);
				//cout << xList[i] - width / 2 << "  " << yList[j] - height / 2 << "  " << width << "  " << height << endl;
				//cout << kf.rect.x << "  " << kf.rect.y << "   " << kf.rect.x + kf.rect.width << "  " << kf.rect.y + kf.rect.height << endl;
				if (kf.rect.x < 0)
				{
					//cout << "layer: " << level << "  " << i << "  " << j << " xleft" << endl;
					//cout << kf.rect.x << endl;
					kf.rect.x = 0;
				}
				if (kf.rect.y < 0)
				{
					//cout << "layer: " << level << "  " << i << "  " << j << " ytop" << endl;
					//cout << kf.rect.y << endl;
					kf.rect.y = 0;
				}
				if (kf.rect.x + kf.rect.width>oriWidth)
				{
					//cout << "layer: " << level << "  " << i << "  " << j << " xright" << endl;
					//cout << kf.rect.x + kf.rect.width << "  " << oriWidth << endl;
					kf.rect.x -= kf.rect.x + kf.rect.width - oriWidth;
				}
				if (kf.rect.y + kf.rect.height > oriHeight)
				{
					//cout << "layer: " << level << "  " << i << "  " << j << " ybottom" << endl;
					//cout << kf.rect.y + kf.rect.height << "  " << oriHeight << endl;
					kf.rect.y -= kf.rect.y + kf.rect.height - oriHeight;
				}
				layer.keyframeList.push_back(kf);
			}
		}
	}
	for (int i = 0; i < layer.keyframeList.size(); i++)
	{
		makeKeyFrame(img, layer.keyframeList[i].rect, layer.keyframeList[i]);
	}
	cout << "layer: " << lev << " end" << endl;
	cout << "+++++++++++++++++++++++++++++++++++++++++" << endl;

}
void PatternDetector::makeKeyFrameList(const Mat& img, vector<KeyFrame>& keyFrameList)
{
	Mat ori = img.clone();
	Size oriSize = ori.size();
	//Size ltSize = Size(oriSize.width / 2, oriSize.height / 2);
	Rect oriRect = Rect(0, 0, oriSize.width, oriSize.height);
	Rect ltRect = Rect(0, 0, oriSize.width / 2, oriSize.height / 2);
	Rect rtRect = Rect(oriSize.width / 2, 0, oriSize.width / 2, oriSize.height / 2);
	Rect lbRect = Rect(0, oriSize.height / 2, oriSize.width / 2, oriSize.height / 2);
	Rect rbRect = Rect(oriSize.width / 2, oriSize.height / 2, oriSize.width / 2, oriSize.height / 2);
	Mat lt, rt, lb, rb;
	ori(ltRect).copyTo(lt);
	ori(rtRect).copyTo(rt);
	ori(lbRect).copyTo(lb);
	ori(rbRect).copyTo(rb);
	vector<Mat> imageList;
	vector<Rect> rectList;
	imageList.push_back(ori);
	imageList.push_back(lt);
	imageList.push_back(rt);
	imageList.push_back(lb);
	imageList.push_back(rb);

	rectList.push_back(oriRect);
	rectList.push_back(ltRect);
	rectList.push_back(rtRect);
	rectList.push_back(lbRect);
	rectList.push_back(rbRect);
	keyFrameList.clear();
	for (int i = 0; i < imageList.size(); i++)
		//for (int i = 4; i < 5; i++)
	{
		KeyFrame kf;
		makeKeyFrame(imageList[i], rectList[i], oriSize, kf);
		keyFrameList.push_back(kf);
	}
	cout << "keyframe information" << endl;
	for (int i = 0; i < keyFrameList.size(); i++)
	{
		cout << "keyframe: " << i << endl;
		cout << keyFrameList[i].keypoints.size() << "  " << keyFrameList[i].descriptors.size() << endl;
	}

	double scale = getPictureDivWindowScale(img.cols, img.rows);
	int level = log(scale) / log(2);
	vector<Rect> rectList1;
	cutImage(ori, level, rectList1);
	cout << "rect list : " << rectList1.size() << endl;
	for (int i = 0; i < rectList1.size(); i++)
	{
		cout << rectList1[i].x << "  " << rectList1[i].y << "  " << rectList1[i].width << "  " << rectList1[i].height << endl;
	}


}

void PatternDetector::makeLayerList(const Mat& image, vector<Layer>& layerList, int layers)
{
	layerList.resize(layers);
	for (int i = 0; i < layerList.size(); i++)
	{
		cout << "make layer list start: " << i << endl;
		makeKeyFrameList(image, layerList[i], i);
		cout << "make layer list end: " << i << endl;
	}
}

void rectToVector(Rect& rect, vector<Point2f>& pointList)
{
	pointList.clear();
	pointList.push_back(Point2f(rect.x, rect.y));
	pointList.push_back(Point2f(rect.x, rect.y + rect.height));
	pointList.push_back(Point2f(rect.x + rect.width, rect.y + rect.height));
	pointList.push_back(Point2f(rect.x + rect.width, rect.y));
}

void draw2dContourWithoutPerpective(Mat& image, vector<Point2f>& points2d, Scalar color, int lineWidth)
{
	for (size_t i = 0; i < points2d.size(); i++)
	{
		line(image, points2d[i], points2d[(i + 1) % points2d.size()], color, lineWidth, CV_AA);
	}
}

void PatternDetector::buildPatternFromImage(const Mat& image, Pattern& pattern)
{
	//if(isMultiScale)
	{
		bool useNewScale = true;
#ifdef WIN32
		useNewScale = false;
#endif
		if (useNewScale)
		{
			int layerNum = getLayerNum(image.cols, image.rows);
			cout << "layer num: " << layerNum << endl;
			makeLayerList(image, pattern.layerList, layerNum);

			//to put all keyframes in a vector
			pattern.keyframeList.clear();
			for (int i = 0; i < pattern.layerList.size(); i++)
			{
				Layer& layer = pattern.layerList[i];
				for (int j = 0; j < layer.keyframeList.size(); j++)
				{
					pattern.keyframeList.push_back(layer.keyframeList[j]);
				}
			}
			if (isMultiScale == false)
			{
				cout << "keyframe list size: " << pattern.keyframeList.size() << endl;
				if (pattern.keyframeList.size() == 1)
				{
					//cout << "keyframe list size: " << pattern.keyframeList.size() << endl;
					cout << "success" << endl;
				}
				else
				{
					cout << "error" << endl;
				}
			}

			for (int i = 0; i < pattern.layerList.size(); i++)
			{
				cout << "layer: " << i << " has " << pattern.layerList[i].keyframeList.size() << " keyframes";
			}
			cout << "all keyframe num is: " << pattern.keyframeList.size() << endl;


			for (int i = 0; i < pattern.layerList.size(); i++)
			{
				Mat layerImg = image.clone();
				//cout << "layer: " << i << endl;
				Layer& layer = pattern.layerList[i];
				for (int j = 0; j < layer.keyframeList.size(); j++)
				{
					//cout << "keyframe: " << j << endl;
					KeyFrame& kf = layer.keyframeList[j];
					vector<Point2f> pointList;
					rectToVector(kf.rect, pointList);
					//if (i == 4)
					draw2dContourWithoutPerpective(layerImg, pointList, Scalar(200, 0, 0), 3);
				}
				string filename = "/sdcard/keyframes/contoursrect_" + intToString(i) + ".jpg";
				imwrite(filename, layerImg);

				for (int j = 0; j < layer.keyframeList.size(); j++)
				{
					//cout << "keyframe: " << j << endl;
					KeyFrame& kf = layer.keyframeList[j];
					vector<Point2f> pointList = kf.points2d;
					//if (i == 4)
					draw2dContourWithoutPerpective(layerImg, pointList, Scalar(200, 0, 0), 3);
				}
				filename = "/sdcard/keyframes/contourspoint_" + intToString(i) + ".jpg";
				imwrite(filename, layerImg);
			}
		}
		else
		{
			makeKeyFrameList(image, pattern.keyframeList);
			pattern.keyframeIndex = 0;
		}

		//init models
		cout << "init models: " << kfmodels.size() << endl;
		cout << pattern.keyframeList.size() << endl;
		//kfmodels.resize(pattern.keyframeList.size());
		cout << kfmodels.size() << endl;
		int meshNum = modelPathList.size();
		cout << meshNum << endl;
		for (int i = 0; i<pattern.keyframeList.size(); i++)
		{
			int pathIndex=i%modelPathList.size();

			Model model(modelPathList[pathIndex]);

			//kfmodels[i].pmesh=makeMesh(modelPathList[ pathIndex]);
			kfmodels.push_back(model);
		}
		cout << kfmodels.size() << endl;
		for (int i = 0; i < pattern.layerList.size(); i++)
		{
			for (int j = 0; j < pattern.layerList[i].keyframeList.size(); j++)
			{
				string filename = "/sdcard/keyframes/" + intToString(i) + "__" + ".jpg";
				imwrite(filename, pattern.layerList[i].keyframeList[j].grayImg);
			}
		}
	}
	//else
	{
	int numImages = 4;
	float step = sqrtf(2.0f);

	// Store original image in pattern structure
	pattern.size = Size(image.cols, image.rows);
	pattern.frame = image.clone();
	getGray(image, pattern.grayImg);

	// Build 2d and 3d contours (3d contour lie in XY plane since it's planar)
	pattern.points2d.resize(4);
	pattern.points3d.resize(4);

	// Image dimensions
	const float w = image.cols;
	const float h = image.rows;

	// Normalized dimensions:
	const float maxSize = (max)(w, h);
	const float unitW = w / maxSize;
	const float unitH = h / maxSize;

	pattern.points2d[0] = Point2f(0, 0);
	pattern.points2d[1] = Point2f(w, 0);
	pattern.points2d[2] = Point2f(w, h);
	pattern.points2d[3] = Point2f(0, h);

	pattern.points3d[0] = Point3f(-unitW, -unitH, 0);
	pattern.points3d[1] = Point3f(unitW, -unitH, 0);
	pattern.points3d[2] = Point3f(unitW, unitH, 0);
	pattern.points3d[3] = Point3f(-unitW, unitH, 0);

	extractFeatures(pattern.grayImg, pattern.keypoints, pattern.descriptors);
}
}



bool PatternDetector::findPatternFirstStage(Mat& image, PatternTrackingInfo& info)
{
	cout << "find pattern first stage" << endl;
	Timer timer;
	timer.start();
	double getGrayStart = timer.getElapsedTimeInMilliSec();
	getGray(image, m_grayImg);
	double getGrayEnd = timer.getElapsedTimeInMilliSec();
	double getGrayDuration = getGrayEnd - getGrayStart;
	cout << "get gray: " << getGrayDuration << endl;


	int maxLostFrames = 5;
	m_roughHomography = info.homography;
	bool homographyFound = false;
	//LOGE("before get branch cols:%d, rows:%d",info.homography.cols,info.homography.rows);
	//homographyFound=false;
	/*if (estimatedHomographyFound && enableWrap)
	{

	LOGE("homography found first");
	//LOGE("type:%d, rows:%d, cold:%d",m_roughHomography.type(),m_roughHomography.rows,m_roughHomography.cols);
	// Warp image using found homography
	double warpStart=timer.getElapsedTimeInMilliSec();
	warpPerspective(m_grayImg, m_firstStageImg, m_roughHomography, m_pattern.size, WARP_INVERSE_MAP );
	//warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_pattern.size, WARP_INVERSE_MAP | INTER_CUBIC);
	double warpEnd=timer.getElapsedTimeInMilliSec();
	double warpDuration=warpEnd-warpStart;
	cout<<"warp image: "<<warpDuration<<endl;
	}
	else*/
	{
		cout << "homography not found first" << endl;
		m_firstStageImg = m_grayImg;
	}
	m_firstToSecondMutex.lock();
	m_firstToSecondImg = m_firstStageImg;
	m_firstToSecondMutex.unlock();
	return true;
}

bool PatternDetector::findPatternSecondStage(Mat& image, PatternTrackingInfo& info, Mat& descriptors)
{
	// Convert input image to gray
	cout << "find pattern second stage" << endl;
	Timer timer;
	timer.start();

	m_firstToSecondMutex.lock();
	m_secondStageImg = m_firstToSecondImg;
	m_firstToSecondMutex.unlock();

	int maxLostFrames = 5;
	m_roughHomography = info.homography;
	bool homographyFound = false;
	//LOGE("before get branch cols:%d, rows:%d",info.homography.cols,info.homography.rows);
	//homographyFound=false;
	/*if (estimatedHomographyFound && enableWrap)
	{
	LOGE("homography found second");
	// Get refined matches:
	//vector<KeyPoint> warpedKeypoints;
	//vector<DMatch> warpedMatches;

	// Detect features on warped image
	double extractStart=timer.getElapsedTimeInMilliSec();
	extractFeatures(m_secondStageImg, m_queryKeypoints, m_queryDescriptors);
	double extractEnd=timer.getElapsedTimeInMilliSec();
	double extractDuration=extractEnd-extractStart;
	cout<<"extract feature: "<<extractDuration<<endl;

	// Match with pattern
	double matchStart=timer.getElapsedTimeInMilliSec();
	getMatches(m_queryDescriptors, m_matches);
	double matchEnd=timer.getElapsedTimeInMilliSec();
	double matchDuration=matchEnd-matchStart;
	cout<<"match feature: "<<matchDuration<<endl;



	}
	else*/
	{
		cout << "homography not found second" << endl;
		// Extract feature points from input gray image
		Timer timer;
		timer.start();
		double extractStart = timer.getElapsedTimeInMilliSec();
		extractFeatures(m_secondStageImg, m_queryKeypoints, m_queryDescriptors);
		double extractEnd = timer.getElapsedTimeInMilliSec();
		double extractDuration = extractEnd - extractStart;
		cout << "extract: " << extractDuration << endl;

		// Get matches with current pattern
		double matchStart = timer.getElapsedTimeInMilliSec();
		getMatches(m_queryDescriptors, m_matches);
		cout << "match size: " << m_matches.size() << endl;
		double matchEnd = timer.getElapsedTimeInMilliSec();
		double matchDuration = matchEnd - matchStart;
		//usleep(60*1000);
		// Find homography transformation and detect good matches

	}


	//return homographyFound;
	return true;
}


bool PatternDetector::findPatternThirdStage(Mat& image, PatternTrackingInfo& info, Mat& descriptors)
{
	// Convert input image to gray
	cout << "find pattern second stage" << endl;
	Timer timer;
	timer.start();



	int maxLostFrames = 5;
	m_roughHomography = info.homography;
	bool homographyFound = false;
	//LOGE("before get branch cols:%d, rows:%d",info.homography.cols,info.homography.rows);
	//homographyFound=false;
	/*if (estimatedHomographyFound && enableWrap)
	{
	LOGE("homography found second");
	// Get refined matches:
	double homographyStart=timer.getElapsedTimeInMilliSec();
	homographyFound = refineMatchesWithHomography(
	m_queryKeypoints,
	m_pattern.keypoints,
	homographyReprojectionThreshold,
	m_matches,
	m_refinedHomography);
	double homographyEnd=timer.getElapsedTimeInMilliSec();
	double homographyDuration=homographyEnd-homographyStart;
	cout<<"homography : "<<homographyDuration<<endl;



	// Get a result homography as result of matrix product of refined and rough homographies:
	if(homographyFound)
	{

	Mat mulHomography=m_roughHomography*m_refinedHomography;//don't know why but can't use info.homography = m_roughHomography * m_refinedHomography
	info.homography = mulHomography;


	//printMat("info.homography",info.homography);
	//printMat("rough homography", m_roughHomography);
	//printMat("refine homography",m_refinedHomography);
	//printMat("mul homography", mul);


	//LOGE("draw rough homography");
	perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
	info.draw2dContour(image, CV_RGB(0,200,0));

	// Transform contour with precise homography
	//LOGE("draw refine homography");
	perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
	//#if _DEBUG
	info.draw2dContour(image, CV_RGB(200,0,200));
	m_lostFrameNum=0;
	estimatedHomographyFound=true;
	}
	else
	{
	info.homography=m_roughHomography;
	m_lostFrameNum++;
	if(m_lostFrameNum>=maxLostFrames){
	estimatedHomographyFound=false;
	}
	}


	m_estimatedHomography=info.homography;

	}
	else*/
	{
		cout << "homography not found second";
		cout << "homography not found" << endl;
		// Extract feature points from input gray image
		Timer timer;
		timer.start();

		// Find homography transformation and detect good matches
		double homographyStart = timer.getElapsedTimeInMilliSec();
		homographyFound = refineMatchesWithHomography(
			m_queryKeypoints,
			m_pattern.keypoints,
			homographyReprojectionThreshold,
			m_matches,
			m_roughHomography);
		double homographyEnd = timer.getElapsedTimeInMilliSec();
		double homographyDuration = homographyEnd - homographyStart;
		cout << "homography: " << homographyDuration << endl;

		cout << "homography found: " << homographyFound << endl;
		// Transform contour with rough homography
		if (homographyFound){

			info.homography = m_roughHomography;

			m_estimatedHomography = info.homography;



			perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
			info.draw2dContour(image, CV_RGB(200, 200, 200));
			m_lostFrameNum = 0;
			estimatedHomographyFound = true;
		}
		else
		{
			m_lostFrameNum++;
			if (m_lostFrameNum >= maxLostFrames){
				estimatedHomographyFound = false;
			}
		}
	}


	return homographyFound;
}

bool PatternDetector::needNewPoints()
{
	//return points[2].size()<minNum || m_opticalFrameNum>100;
	return after.size()<minNum || m_opticalFrameNum>updateFrameNum;
}

bool PatternDetector::compareAlpha(AlphaState a, AlphaState b)
{
	return a.alpha > b.alpha;
}

bool PatternDetector::compareCount(MatchState a, MatchState b)
{
	if (a.found&&!b.found)
	{
		return true;
	}
	else if (b.found&&!a.found)
	{
		return false;
	}
	else
	{
		return a.num > b.num;
	}
}

double PatternDetector::calcWindowArea()
{
	MyPoint lt(0, 0), lb(0, 600), rb(800, 600), rt(800, 0);
	vector<MyPoint> screenPointList;
	screenPointList.push_back(lt);
	screenPointList.push_back(lb);
	screenPointList.push_back(rb);
	screenPointList.push_back(rt);
	MyPolygon screenPoly(screenPointList);
	double screenArea = screenPoly.area_of_polygon(screenPoly.pointList);
	cout << "in calc window area: " << endl;
	cout << screenArea << endl;
	return screenArea;
}

MyPolygon PatternDetector::makeScreenPoly()
{
	MyPoint lt(0, 0), lb(0, screenHeight), rb(screenWidth, screenHeight), rt(screenWidth, 0);
	vector<MyPoint> screenPointList;
	screenPointList.push_back(lt);
	screenPointList.push_back(lb);
	screenPointList.push_back(rb);
	screenPointList.push_back(rt);
	MyPolygon screenPoly(screenPointList);
	double screenArea = screenPoly.area_of_polygon(screenPoly.pointList);
	cout << "screea area" << endl;
	cout << screenArea << endl;
	return screenPoly;
}

bool PatternDetector::matchKeyframesWithPolygon(Mat& homography, vector<int>& indexes)
{
	indexes.clear();
	string str;
	if (isMultiScale)
	{

		if (estimatedHomographyFound)
		{
			cout << "begin calc areas" << endl;
			/*MyPoint lt(0, 0), lb(0, screenHeight), rb(screenWidth, screenHeight), rt(screenWidth, 0);
			vector<MyPoint> screenPointList;
			screenPointList.push_back(lt);
			screenPointList.push_back(lb);
			screenPointList.push_back(rb);
			screenPointList.push_back(rt);
			MyPolygon screenPoly(screenPointList);
			double screenArea = screenPoly.area_of_polygon(screenPoly.pointList);
			cout << "screea area" << endl;
			cout << screenArea << endl;*/
			double screenArea = screenPolygon.area_of_polygon(screenPolygon.pointList);
			vector<AlphaState> alphaList;
			bool success = true;
			Timer timer;
			timer.start();
			double polyStart = timer.getElapsedTimeInMilliSec();
			for (int i = 0; i < m_pattern.keyframeList.size(); i++)
			{
				vector<MyPoint> keyframePointList;
				PatternTrackingInfo info;
				info.homography = homography;
				perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, info.homography);
				for (int j = 0; j < 4; j++)
				{
					MyPoint p(info.points2d[j].x, info.points2d[j].y);
					keyframePointList.push_back(p);
				}
				MyPolygon keyframePoly(keyframePointList);
				if (!keyframePoly.issimple() || !keyframePoly.isconvex())
				{
					success = false;
					break;
				}
				MyPolygon interPoly;
				bool success = screenPolygon.intersect(screenPolygon, keyframePoly, interPoly);
				double keyframeArea = keyframePoly.area_of_polygon(keyframePoly.pointList);
				double interArea = interPoly.area_of_polygon(interPoly.pointList);
				double alpha = interArea / screenArea + interArea / keyframeArea;
				AlphaState as;
				as.alpha = alpha;
				as.index = i;
				alphaList.push_back(as);
				cout << "keyframe " << i << endl;
				cout << keyframeArea << endl;
				cout << "inter area" << endl;
				cout << interArea << endl;
				cout << "alpha" << endl;
				cout << alpha << endl;
			}
			double polyEnd = timer.getElapsedTimeInMilliSec();
			double polyDuration = polyEnd - polyStart;
			cout << "calc poly time: " << polyDuration << endl;

			if (success)
			{
				double estiSortStart = timer.getElapsedTimeInMilliSec();
				int estiIndex = -1;
				double max = 0;
				for (int i = 0; i < alphaList.size(); i++)
				{
					if (alphaList[i].alpha>max)
					{
						max = alphaList[i].alpha;
						estiIndex = i;
					}
				}
				cout << "before sort esti" << endl;
				for (int i = 0; i < alphaList.size(); i++)
				{
					cout << alphaList[i].alpha << "  " << alphaList[i].index << endl;
				}
				sort(alphaList.begin(), alphaList.end(), compareAlpha);
				cout << "after sort esti" << endl;
				for (int i = 0; i < alphaList.size(); i++)
				{
					cout << alphaList[i].alpha << "  " << alphaList[i].index << endl;
				}
				int resultCount = (min)((int)alphaList.size(), indexCount);
				for (int i = 0; i < resultCount; i++)
				{
					indexes.push_back(alphaList[i].index);
				}
				stringstream ss;
				for (int i = 0; i < indexes.size(); i++)
				{
					ss << indexes[i] << "  ";
				}
				str = ss.str();
				double estiSortEnd = timer.getElapsedTimeInMilliSec();
				double estiSortDuration = estiSortEnd - estiSortStart;
				cout << "esti sort duration" << estiSortDuration << endl;

				//copy vector
				//indexes = estiIdxes;
				return success;
			}
		}
	}
	else //single scale
	{
		indexes.push_back(0);
	}
}

int PatternDetector::matchKeyFrames(Mat& homography, vector<int>& indexes, vector<int>& matchIdxes, vector<int>& estiIdxes, string& str)
{
	indexes.clear();
	matchIdxes.clear();
	estiIdxes.clear();
	if (estimatedHomographyFound)
	{
		cout << "begin calc areas" << endl;
		MyPoint lt(0, 0), lb(0, screenHeight), rb(screenWidth, screenHeight), rt(screenWidth, 0);
		vector<MyPoint> screenPointList;
		screenPointList.push_back(lt);
		screenPointList.push_back(lb);
		screenPointList.push_back(rb);
		screenPointList.push_back(rt);
		MyPolygon screenPoly(screenPointList);
		double screenArea = screenPoly.area_of_polygon(screenPoly.pointList);
		cout << "screea area" << endl;
		cout << screenArea << endl;
		vector<AlphaState> alphaList;
		bool success = true;
		Timer timer;
		timer.start();
		double polyStart = timer.getElapsedTimeInMilliSec();
		for (int i = 0; i < m_pattern.keyframeList.size(); i++)
		{
			vector<MyPoint> keyframePointList;
			PatternTrackingInfo info;
			info.homography = homography;
			perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, info.homography);
			for (int j = 0; j < 4; j++)
			{
				MyPoint p(info.points2d[j].x, info.points2d[j].y);
				keyframePointList.push_back(p);
			}
			MyPolygon keyframePoly(keyframePointList);
			if (!keyframePoly.issimple() || !keyframePoly.isconvex())
			{
				success = false;
				break;
			}
			MyPolygon interPoly;
			bool success = screenPoly.intersect(screenPoly, keyframePoly, interPoly);
			double keyframeArea = keyframePoly.area_of_polygon(keyframePoly.pointList);
			double interArea = interPoly.area_of_polygon(interPoly.pointList);
			double alpha = interArea / screenArea + interArea / keyframeArea;
			AlphaState as;
			as.alpha = alpha;
			as.index = i;
			alphaList.push_back(as);
			cout << "keyframe " << i << endl;
			cout << keyframeArea << endl;
			cout << "inter area" << endl;
			cout << interArea << endl;
			cout << "alpha" << endl;
			cout << alpha << endl;
		}
		double polyEnd = timer.getElapsedTimeInMilliSec();
		double polyDuration = polyEnd - polyStart;
		cout << "calc poly time: " << polyDuration << endl;

		if (success)
		{
			double estiSortStart = timer.getElapsedTimeInMilliSec();
			int estiIndex = -1;
			double max = 0;
			for (int i = 0; i < alphaList.size(); i++)
			{
				if (alphaList[i].alpha>max)
				{
					max = alphaList[i].alpha;
					estiIndex = i;
				}
			}
			cout << "before sort esti" << endl;
			for (int i = 0; i < alphaList.size(); i++)
			{
				cout << alphaList[i].alpha << "  " << alphaList[i].index << endl;
			}
			sort(alphaList.begin(), alphaList.end(), compareAlpha);
			cout << "after sort esti" << endl;
			for (int i = 0; i < alphaList.size(); i++)
			{
				cout << alphaList[i].alpha << "  " << alphaList[i].index << endl;
			}
			int resultCount = (min)((int)alphaList.size(), indexCount);
			for (int i = 0; i < resultCount; i++)
			{
				indexes.push_back(alphaList[i].index);
				estiIdxes.push_back(alphaList[i].index);
			}
			stringstream ss;
			for (int i = 0; i < indexes.size(); i++)
			{
				ss << indexes[i] << "  ";
			}
			str = ss.str();
			double estiSortEnd = timer.getElapsedTimeInMilliSec();
			double estiSortDuration = estiSortEnd - estiSortStart;
			cout << "esti sort duration" << estiSortDuration << endl;

			//copy vector
			//indexes = estiIdxes;
			return estiIndex;
		}
	}
	//else //上次没找到或者这次没找到都跳转到这里来 使用匹配算法寻找关键帧
	{
		extractFeatures(m_grayImg, m_queryKeypoints, m_queryDescriptors);
		int size = m_pattern.keyframeList.size();
		vector<MatchState> matchCount;
		vector<vector<DMatch> > matchesList(size);
		Timer timer;
		timer.start();
		double matchStart = timer.getElapsedTimeInMilliSec();
		cout << "begin match" << endl;
		for (int i = 0; i < size; i++){
			cout << "begin match keyframe: " << i << endl;
			MatchState ms = matchKeyFrame(i, matchesList[i]);
			cout << ms.found << ms.num << ms.index;
			matchCount.push_back(ms);
		}
		double matchEnd = timer.getElapsedTimeInMilliSec();
		double matchDuration = matchEnd - matchStart;
		cout << "calc match time: " << matchDuration << endl;

		stringstream ss;
		ss << "match condition: ";
		cout << "match count; " << matchCount.size() << endl;
		for (int i = 0; i < matchCount.size(); i++)
		{
			ss << matchCount[i].found << "  " << matchCount[i].num << "  ";
		}
		str = ss.str();
		cout << "ss: " << str << endl;

		int count = 0;
		int noEstiIndex = -1;
		for (int i = 0; i<size; i++){
			if (matchCount[i].found){
				if (matchCount[i].num >= 8 && matchCount[i].num > count){
					count = matchCount[i].num;
					noEstiIndex = i;
				}
			}
		}
		cout << "before sort match" << matchCount.size() << endl;
		for (int i = 0; i < matchCount.size(); i++)
		{
			cout << matchCount[i].found << "  " << matchCount[i].num << "  " << matchCount[i].index << endl;
		}
		sort(matchCount.begin(), matchCount.end(), compareCount);
		cout << "after sort match" << matchCount.size() << endl;
		for (int i = 0; i < matchCount.size(); i++)
		{
			cout << matchCount[i].found << "  " << matchCount[i].num << "  " << matchCount[i].index << endl;
		}
		int resultCount = (min)((int)matchCount.size(), indexCount);
		cout << "result count" << resultCount << endl;

		for (int i = 0; i < resultCount; i++)
		{
			if (matchCount[i].found == false)
			{
				break;
			}
			//indexes.push_back(matchCount[i].index);
			matchIdxes.push_back(matchCount[i].index);
		}

		//copy vector
		indexes = matchIdxes;
		//cout << "indexes size: " <<indexes.size()<< endl;
		//for (int i = 0; i < indexes.size(); i++)
		//{
		//	cout << indexes[i] << "  ";
		//}
		//cout << endl;

		//cout << "matchIdxes size: " <<matchIdxes.size()<< endl;
		//for (int i = 0; i < matchIdxes.size(); i++)
		//{
		//	cout << matchIdxes[i] << "  ";
		//}
		//cout << endl;

		//cout << "estiIdxes size: " << estiIdxes.size() << endl;
		//for (int i = 0; i < estiIdxes.size(); i++)
		//{
		//	cout << estiIdxes[i] << "  ";
		//}
		//cout << endl;

		return noEstiIndex;
	}

}

int PatternDetector::matchKeyFramesNew(Mat& homography, vector<int>& indexes, vector<int>& matchIdxes, vector<int>& estiIdxes, string& str)
{
	indexes.clear();
	matchIdxes.clear();
	estiIdxes.clear();
	if (estimatedHomographyFound)
	{
		cout << "begin calc areas" << endl;
		MyPoint lt(0, 0), lb(0, screenHeight), rb(screenWidth, screenHeight), rt(screenWidth, 0);
		vector<MyPoint> screenPointList;
		screenPointList.push_back(lt);
		screenPointList.push_back(lb);
		screenPointList.push_back(rb);
		screenPointList.push_back(rt);
		MyPolygon screenPoly(screenPointList);
		double screenArea = screenPoly.area_of_polygon(screenPoly.pointList);
		cout << "screea area" << endl;
		cout << screenArea << endl;
		vector<AlphaState> alphaList;
		bool success = true;
		Timer timer;
		timer.start();
		double polyStart = timer.getElapsedTimeInMilliSec();
		for (int i = 0; i < m_pattern.keyframeList.size(); i++)
		{
			vector<MyPoint> keyframePointList;
			PatternTrackingInfo info;
			info.homography = homography;
			perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, info.homography);
			for (int j = 0; j < 4; j++)
			{
				MyPoint p(info.points2d[j].x, info.points2d[j].y);
				keyframePointList.push_back(p);
			}
			MyPolygon keyframePoly(keyframePointList);
			if (!keyframePoly.issimple() || !keyframePoly.isconvex())
			{
				success = false;
				break;
			}
			MyPolygon interPoly;
			bool success = screenPoly.intersect(screenPoly, keyframePoly, interPoly);
			double keyframeArea = keyframePoly.area_of_polygon(keyframePoly.pointList);
			double interArea = interPoly.area_of_polygon(interPoly.pointList);
			double alpha = interArea / screenArea + interArea / keyframeArea;
			AlphaState as;
			as.alpha = alpha;
			as.index = i;
			alphaList.push_back(as);
			cout << "keyframe " << i << endl;
			cout << keyframeArea << endl;
			cout << "inter area" << endl;
			cout << interArea << endl;
			cout << "alpha" << endl;
			cout << alpha << endl;
		}
		double polyEnd = timer.getElapsedTimeInMilliSec();
		double polyDuration = polyEnd - polyStart;
		cout << "calc poly time: " << polyDuration << endl;

		if (success)
		{
			double estiSortStart = timer.getElapsedTimeInMilliSec();
			int estiIndex = -1;
			double max = 0;
			for (int i = 0; i < alphaList.size(); i++)
			{
				if (alphaList[i].alpha>max)
				{
					max = alphaList[i].alpha;
					estiIndex = i;
				}
			}
			cout << "before sort esti" << endl;
			for (int i = 0; i < alphaList.size(); i++)
			{
				cout << alphaList[i].alpha << "  " << alphaList[i].index << endl;
			}
			sort(alphaList.begin(), alphaList.end(), compareAlpha);
			cout << "after sort esti" << endl;
			for (int i = 0; i < alphaList.size(); i++)
			{
				cout << alphaList[i].alpha << "  " << alphaList[i].index << endl;
			}
			int resultCount = (min)((int)alphaList.size(), indexCount);
			for (int i = 0; i < resultCount; i++)
			{
				indexes.push_back(alphaList[i].index);
				estiIdxes.push_back(alphaList[i].index);
			}
			stringstream ss;
			for (int i = 0; i < indexes.size(); i++)
			{
				ss << indexes[i] << "  ";
			}
			str = ss.str();
			double estiSortEnd = timer.getElapsedTimeInMilliSec();
			double estiSortDuration = estiSortEnd - estiSortStart;
			cout << "esti sort duration" << estiSortDuration << endl;

			//copy vector
			//indexes = estiIdxes;
			return estiIndex;
		}
	}
	//else //上次没找到或者这次没找到都跳转到这里来 使用匹配算法寻找关键帧
	{
		for (int i = 0; i < m_pattern.keyframeList.size(); i++)
		{
			indexes.push_back(i);
			matchIdxes.push_back(i);
		}
	}

}

MatchState PatternDetector::matchKeyFrame(int index, vector<DMatch>& matches)
{
	Timer timer;
	timer.start();

	KeyFrame& keyframe = m_pattern.keyframeList[index];
	// Get matches with current pattern
	double matchStart = timer.getElapsedTimeInMilliSec();
	getMatches(m_queryDescriptors, index, matches);
	cout << "after get match : " << matches.size() << endl;
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;

	// Find homography transformation and detect good matches
	double homographyStart = timer.getElapsedTimeInMilliSec();
	bool homographyFound = refineMatchesWithHomography(
		m_queryKeypoints,
		keyframe.keypoints,
		homographyReprojectionThreshold,
		matches,
		m_roughHomography);
	cout << "after homography: " << matches.size() << endl;
	double homographyEnd = timer.getElapsedTimeInMilliSec();
	double homographyDuration = homographyEnd - homographyStart;
	cout << "homography: " << homographyDuration << endl;
	MatchState ms;
	ms.found = homographyFound;
	ms.num = matches.size();
	ms.index = index;
	return ms;
}

bool PatternDetector::OpticalTracking(Mat& image, PatternTrackingInfo& info)
{
	cout << "begin optical tracking" << endl;
	bool homographyFound = false;
	// 2. track features
	cout << "before optical flow: " << before.size() << endl;
	cout << "before optical flow: " << before.size() << endl;
	InputArray _prevPts = (InputArray)before;
	Mat prevPtsMat = _prevPts.getMat();

	int npoints = prevPtsMat.checkVector(2, CV_32F, true);
	//cout<<npoints<<endl;
	cout << npoints << endl;
	calcOpticalFlowPyrLK(m_grayImgPrev, m_grayImg, // 2 consecutive images
		before, // input point position in first image
		after, // output point postion in the second image
		status,    // tracking success
		err);      // tracking error

	// 2. loop over the tracked points to reject the undesirables
	int k = 0;
	for (int i = 0; i < after.size(); i++) {

		// do we keep this point?
		if (acceptTrackedPoint(i)) {

			// keep this point in vector
			patternPoints[k] = patternPoints[i];
			initial[k] = initial[i];
			after[k++] = after[i];
		}
	}

	// eliminate unsuccesful points
	patternPoints.resize(k);
	after.resize(k);
	initial.resize(k);
	cout << "k: " << k << endl;
	Mat homography;
	if (k >= 4)
	{
		// Find homography matrix and get inliers mask
		vector<unsigned char> inliersMask(before.size());
		Timer timer;
		timer.start();
		double findHomographyStart = timer.getElapsedTimeInMilliSec();
		homography = findHomography(patternPoints,
			after,
			0,
			homographyReprojectionThreshold,
			inliersMask);

		double findHomographyEnd = timer.getElapsedTimeInMilliSec();
		double findHomographyDuration = findHomographyEnd - findHomographyStart;
		cout << "find homography duration: " << findHomographyDuration << endl;

		homographyFound = (countNonZero(inliersMask) > 8);
	}
	else
	{
		homographyFound = false;
	}



	
	if (homographyFound){
		info.homography = homography;


		//printMat("info.homography",info.homography);
		//printMat("rough homography", m_roughHomography);
		//printMat("refine homography",m_refinedHomography);
		//printMat("mul homography", mul);


		//LOGE("draw rough homography");
		//perspectiveTransform(m_pattern.points2d, info.points2d, m_initialHomography);
		//info.draw2dContour(image, CV_RGB(0,200,0));

		// Transform contour with precise homography
		//LOGE("draw refine homography");
		//perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
		//#if _DEBUG
		//info.draw2dContour(image, CV_RGB(200,0,200));
		//draw2dContour(image,info, m_pattern.points2d,info.homography, CV_RGB(200, 0, 200));
		//drawing.draw2dContour(*this, image, info, m_pattern.points2d, info.homography, CV_RGB(200, 0, 200));
		drawing.drawContours(*this, image, info, nowMatchedKeyframes);
		m_lostFrameNum = 0;
		estimatedHomographyFound = true;
		float err = arerror.computeError(*this, homography);
		cout << "err: " << err << endl;
		// 3. handle the accepted tracked points
		Scalar scalar(255, 0, 0);
		handleTrackedPoints(image, image, scalar);

		//set estimation
		m_estimatedHomography = info.homography;
		m_initialHomography = info.homography;

		// 4. current points and image become previous ones
		swap(before, after);
		swap(m_grayImgPrev, m_grayImg);
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
	}
	m_opticalFrameNum++;
	return homographyFound;
}

void PatternDetector::getKeyPointsAndDescriptors(Mat& image, vector<int> indexes, vector<KeyPoint>& keyPoints, Mat& descriptors)
{
	keyPoints.clear();
	descriptors.release();
	vector<KeyFrame> keyframeList(indexes.size());
	for (int i = 0; i < indexes.size(); i++)
	{
		int index = indexes[i];
		Rect& rect = m_pattern.keyframeList[index].rect;
		makeKeyFrame(image, rect, keyframeList[i]);
	}
	for (int i = 0; i < keyframeList.size(); i++)
	{
		keyPoints.insert(keyPoints.end(), keyframeList[i].keypoints.begin(), keyframeList[i].keypoints.end());
		descriptors.push_back(keyframeList[i].descriptors);
	}
}

bool PatternDetector::warpedTracking(Mat& image, PatternTrackingInfo& info)
{
	cout << "begin warped tracking" << endl;
	bool homographyFound = false;
	//LOGE("homography found");
	//LOGE("type:%d, rows:%d, cold:%d",m_roughHomography.type(),m_roughHomography.rows,m_roughHomography.cols);
	// Warp image using found homography
	Timer timer;
	double warpStart = timer.getElapsedTimeInMilliSec();
	warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_pattern.size, WARP_INVERSE_MAP);
	//warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_pattern.size, WARP_INVERSE_MAP | INTER_CUBIC);
	double warpEnd = timer.getElapsedTimeInMilliSec();
	double warpDuration = warpEnd - warpStart;
	cout << "warp image: " << warpDuration << endl;

	// Get refined matches:
	vector<KeyPoint> warpedKeypoints;
	vector<DMatch> warpedMatches;

	// Detect features on warped image
	double extractStart = timer.getElapsedTimeInMilliSec();
	extractFeatures(m_warpedImg, warpedKeypoints, m_queryDescriptors);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	double extractDuration = extractEnd - extractStart;
	cout << "extract feature: " << extractDuration << endl;

	// Match with pattern
	double matchStart = timer.getElapsedTimeInMilliSec();
	cout << "begin get matches" << endl;

	vector<int> indexes;
	indexes.clear();
	vector<int> estiIndexes;
	vector<int> matchIndexes;
	string str;
	m_pattern.keyframeIndex = matchKeyFrames(m_estimatedHomography, indexes, estiIndexes, matchIndexes, str);

	if (m_pattern.keyframeIndex == -1)
	{
		m_pattern.keyframeIndex = 0;
	}

	getMatches(m_queryDescriptors, indexes, m_matches);
	cout << "end get matches" << endl;
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;
	cout << m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size() << endl;
	cout << m_pattern.keypoints.size() << endl;

	vector<KeyPoint> patternKeyPoints;
	patternKeyPoints.clear();
	for (int i = 0; i < indexes.size(); i++)
	{
		patternKeyPoints.insert(patternKeyPoints.end(), m_pattern.keyframeList[i].keypoints.begin(), m_pattern.keyframeList[i].keypoints.end());
	}





	// Estimate new refinement homography
	double homographyStart = timer.getElapsedTimeInMilliSec();
	homographyFound = refineMatchesWithHomography(
		warpedKeypoints,
		m_pattern.keypoints,
		homographyReprojectionThreshold,
		warpedMatches,
		m_refinedHomography);
	double homographyEnd = timer.getElapsedTimeInMilliSec();
	double homographyDuration = homographyEnd - homographyStart;
	cout << "homography : " << homographyDuration << endl;

	/*if(homographyFound){
	LOGE("refine homography found");
	}
	else{
	LOGE("refine homography not found");
	}*/

	// Get a result homography as result of matrix product of refined and rough homographies:
	if (homographyFound)
	{
		Mat mulHomography = m_roughHomography*m_refinedHomography;//don't know why but can't use info.homography = m_roughHomography * m_refinedHomography
		info.homography = mulHomography;


		//printMat("info.homography",info.homography);
		//printMat("rough homography", m_roughHomography);
		//printMat("refine homography",m_refinedHomography);
		//printMat("mul homography", mul);


		//LOGE("draw rough homography");
		perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
		//info.draw2dContour(image, CV_RGB(0,200,0));

		// Transform contour with precise homography
		//LOGE("draw refine homography");
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
		//#if _DEBUG
		info.draw2dContour(image, CV_RGB(200, 0, 200));
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
		m_lostFrameNum = 0;
		estimatedHomographyFound = true;
	}
	else
	{
		info.homography = m_roughHomography;
		m_lostFrameNum++;
		if (m_lostFrameNum >= maxLostFrames){
			estimatedHomographyFound = false;
		}
	}
	cout << "warped matches size: " << warpedMatches.size() << endl;
	initial.clear();
	before.clear();
	after.clear();
	//before.clear();
	//after.clear();
	for (int i = 0; i < warpedMatches.size(); i++){
		initial.push_back(warpedKeypoints[warpedMatches[i].queryIdx].pt);
		//points[0].push_back(warpedKeypoints[warpedMatches[i].queryIdx].pt);
		before.push_back(warpedKeypoints[warpedMatches[i].queryIdx].pt);
	}
	cout << "begin handle tracked points outer" << endl;
	handleTrackedPoints(image, image);
	m_estimatedHomography = info.homography;
	m_initialHomography = info.homography;
	return homographyFound;
}

bool PatternDetector::warpedTrackingSimple(Mat& image, PatternTrackingInfo& info)
{
	bool homographyFound = false;
	//LOGE("homography found");
	//LOGE("type:%d, rows:%d, cold:%d",m_roughHomography.type(),m_roughHomography.rows,m_roughHomography.cols);
	// Warp image using found homography
	Timer timer;
	double warpSimpleStart = timer.getElapsedTimeInMilliSec();
	double warpStart = timer.getElapsedTimeInMilliSec();
	warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_pattern.size, WARP_INVERSE_MAP);
	//warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_pattern.size, WARP_INVERSE_MAP | INTER_CUBIC);
	double warpEnd = timer.getElapsedTimeInMilliSec();
	double warpDuration = warpEnd - warpStart;
	cout << "warp image: " << warpDuration << endl;

	// Get refined matches:
	vector<KeyPoint> warpedKeypoints;
	vector<DMatch> warpedMatches;

	// Detect features on warped image
	double extractStart = timer.getElapsedTimeInMilliSec();
	Size size = m_warpedImg.size();
	Rect rect(0, 0, size.width, size.height);
	KeyFrame kf;
	makeKeyFrame(m_warpedImg, rect, kf);
	warpedKeypoints = kf.keypoints;
	m_queryDescriptors = kf.descriptors;
	//extractFeatures(m_warpedImg, warpedKeypoints, m_queryDescriptors);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	double extractDuration = extractEnd - extractStart;
	cout << "extract feature: " << extractDuration << endl;

	// Match with pattern
	double matchStart = timer.getElapsedTimeInMilliSec();
	cout << "begin get matches" << endl;

	//vector<int> indexes;
	//indexes.clear();
	vector<int> estiIndexes;
	vector<int> matchIndexes;
	string str;
	//m_pattern.keyframeIndex = matchKeyFrames(m_estimatedHomography, indexes, estiIndexes, matchIndexes, str);

	//vector<KeyFrame> selectedRects(indexes.size());
	//vector<KeyPoint> selectedKeyPoints;
	//Mat selectedDescriptors;
	//for (int i = 0; i < indexes.size(); i++)
	//{
	//	int index = indexes[i];
	//	makeKeyFrame(m_warpedImg, m_pattern.keyframeList[index].rect, selectedRects[i]);
	//}

	//for (int i = 0; i < selectedRects.size(); i++)
	//{
	//	selectedKeyPoints.insert(selectedKeyPoints.end(), selectedRects[i].keypoints.begin(), selectedRects[i].keypoints.end());
	//	selectedDescriptors.push_back(selectedRects[i].descriptors);
	//}
	//m_queryDescriptors = selectedDescriptors;
	//warpedKeypoints = selectedKeyPoints;
	//getKeyPointsAndDescriptors(m_warpedImg, indexes, warpedKeypoints, m_queryDescriptors);
	getMatches(m_queryDescriptors, oriIndex, m_matches);
	cout << "end get matches" << endl;
	//if (indexes.size() != 0)
	//{
	//	nowMatchedKeyframes = indexes;
	//}
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;
	cout << m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size() << endl;
	cout << m_pattern.keypoints.size() << endl;

	vector<KeyPoint> &patternKeyPoints = m_pattern.keyframeList[oriIndex].keypoints;
	//patternKeyPoints.clear();
	//for (int i = 0; i < indexes.size(); i++)
	//{
	//	patternKeyPoints.insert(patternKeyPoints.end(), m_pattern.keyframeList[indexes[i]].keypoints.begin(), m_pattern.keyframeList[indexes[i]].keypoints.end());
	//}





	// Estimate new refinement homography
	double homographyStart = timer.getElapsedTimeInMilliSec();
	homographyFound = refineMatchesWithHomography(
		warpedKeypoints,
		patternKeyPoints,
		homographyReprojectionThreshold,
		m_matches,
		m_refinedHomography);
	double homographyEnd = timer.getElapsedTimeInMilliSec();
	double homographyDuration = homographyEnd - homographyStart;
	cout << "homography : " << homographyDuration << endl;


	if (homographyFound)
	{
		Mat mulHomography = m_roughHomography*m_refinedHomography;//don't know why but can't use info.homography = m_roughHomography * m_refinedHomography
		info.homography = mulHomography;

		m_estimatedHomography = info.homography;
		m_initialHomography = info.homography;


		initial.clear();
		before.clear();
		after.clear();
		patternPoints.clear();
		vector<Point2f> warpedPoints;
		vector<Point2f> queryPoints;
		for (int i = 0; i < m_matches.size(); i++)
		{
			warpedPoints.push_back(warpedKeypoints[m_matches[i].queryIdx].pt);
		}

		perspectiveTransform(warpedPoints, queryPoints, m_roughHomography);
		for (int i = 0; i < m_matches.size(); i++){
			//if(isMultiScale){
			//patternPoints.push_back(m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints[m_matches[i].trainIdx].pt);
			patternPoints.push_back(patternKeyPoints[m_matches[i].trainIdx].pt);
			//}else{
			//	patternPoints.push_back(m_pattern.keypoints[m_matches[i].trainIdx].pt);
			//}
			//initial.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
			//before.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
			//after.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);

			initial = queryPoints;
			before = queryPoints;
			after = queryPoints;
		}
		float err = arerror.computeError(*this, m_roughHomography);
		cout << "end compute error" << endl;


		//printMat("info.homography",info.homography);
		//printMat("rough homography", m_roughHomography);
		//printMat("refine homography",m_refinedHomography);
		//printMat("mul homography", mul);


		//LOGE("draw rough homography");
		perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
		info.draw2dContour(image, CV_RGB(0, 200, 0));

		// Transform contour with precise homography
		//LOGE("draw refine homography");
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
		//#if _DEBUG
		info.draw2dContour(image, CV_RGB(200, 0, 200));
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);


		cout << "begin draw contours" << endl;
		
		//match keyframes
		vector<int> indexes;
		vector<int> matchindexes;
		vector<int> estiindexes;
		string str;
		m_roughHomography = info.homography;
		matchKeyframesWithPolygon(m_roughHomography, indexes);
		//matchKeyFrames(m_roughHomography, indexes,matchindexes,estiindexes,str);
		nowMatchedKeyframes = indexes;

		
		drawing.drawContours(*this, image, info, indexes);
		cout << "end draw contours" << endl;
		//drawing.draw2Contours(*this, image, info, matchIndexes, estiIndexes);
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);


		m_lostFrameNum = 0;
		estimatedHomographyFound = true;



		
	}
	else
	{
		info.homography = m_roughHomography;
		m_lostFrameNum++;
		if (m_lostFrameNum >= maxLostFrames){
			estimatedHomographyFound = false;
		}
	}
	cout << "warped matches size: " << warpedMatches.size() << endl;
	cout << "begin handle tracked points outer" << endl;
	handleTrackedPoints(image, image);
	m_grayImgPrev = m_grayImg.clone();
	double warpSimpleEnd = timer.getElapsedTimeInMilliSec();
	double warpSimpleDuration = warpSimpleEnd - warpSimpleStart;
	cout << "warp simple duration: " << warpSimpleDuration << endl;
	return homographyFound;
}

bool PatternDetector::warpedTrackingNew(Mat& image, PatternTrackingInfo& info)
{
	bool homographyFound = false;
	homographyFound = warpedTrackingSimple(image, info);
	if (homographyFound){
		return true;
	}
	//LOGE("homography found");
	//LOGE("type:%d, rows:%d, cold:%d",m_roughHomography.type(),m_roughHomography.rows,m_roughHomography.cols);
	// Warp image using found homography
	if (true)
	{
		Timer timer;
		double warpStart = timer.getElapsedTimeInMilliSec();
		warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_pattern.size, WARP_INVERSE_MAP);
		//warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_pattern.size, WARP_INVERSE_MAP | INTER_CUBIC);
		double warpEnd = timer.getElapsedTimeInMilliSec();
		double warpDuration = warpEnd - warpStart;
		cout << "warp image: " << warpDuration << endl;

		// Get refined matches:
		vector<KeyPoint> warpedKeypoints;
		vector<DMatch> warpedMatches;

		// Detect features on warped image
		double extractStart = timer.getElapsedTimeInMilliSec();
		//Size size = m_warpedImg.size();
		//Rect rect(0, 0, size.width, size.height);
		//KeyFrame kf;
		//makeKeyFrame(m_warpedImg, rect, kf);
		//warpedKeypoints = kf.keypoints;
		//m_queryDescriptors = kf.descriptors;
		//extractFeatures(m_warpedImg, warpedKeypoints, m_queryDescriptors);
		double extractEnd = timer.getElapsedTimeInMilliSec();
		double extractDuration = extractEnd - extractStart;
		cout << "extract feature: " << extractDuration << endl;

		// Match with pattern
		double matchStart = timer.getElapsedTimeInMilliSec();
		cout << "begin get matches" << endl;

		vector<int> indexes;
		indexes.clear();
		vector<int> estiIndexes;
		vector<int> matchIndexes;
		string str;
		m_pattern.keyframeIndex = matchKeyFrames(m_estimatedHomography, indexes, estiIndexes, matchIndexes, str);

		//vector<KeyFrame> selectedRects(indexes.size());
		//vector<KeyPoint> selectedKeyPoints;
		//Mat selectedDescriptors;
		//for (int i = 0; i < indexes.size(); i++)
		//{
		//	int index = indexes[i];
		//	makeKeyFrame(m_warpedImg, m_pattern.keyframeList[index].rect, selectedRects[i]);
		//}

		//for (int i = 0; i < selectedRects.size(); i++)
		//{
		//	selectedKeyPoints.insert(selectedKeyPoints.end(), selectedRects[i].keypoints.begin(), selectedRects[i].keypoints.end());
		//	selectedDescriptors.push_back(selectedRects[i].descriptors);
		//}
		//m_queryDescriptors = selectedDescriptors;
		//warpedKeypoints = selectedKeyPoints;
		getKeyPointsAndDescriptors(m_warpedImg, indexes, warpedKeypoints, m_queryDescriptors);
		getMatches(m_queryDescriptors, indexes, m_matches);
		cout << "end get matches" << endl;
		if (indexes.size() != 0)
		{
			nowMatchedKeyframes = indexes;
		}
		double matchEnd = timer.getElapsedTimeInMilliSec();
		double matchDuration = matchEnd - matchStart;
		cout << m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size() << endl;
		cout << m_pattern.keypoints.size() << endl;

		vector<KeyPoint> patternKeyPoints;
		patternKeyPoints.clear();
		for (int i = 0; i < indexes.size(); i++)
		{
			patternKeyPoints.insert(patternKeyPoints.end(), m_pattern.keyframeList[indexes[i]].keypoints.begin(), m_pattern.keyframeList[indexes[i]].keypoints.end());
		}





		// Estimate new refinement homography
		double homographyStart = timer.getElapsedTimeInMilliSec();
		homographyFound = refineMatchesWithHomography(
			warpedKeypoints,
			patternKeyPoints,
			homographyReprojectionThreshold,
			m_matches,
			m_refinedHomography);
		double homographyEnd = timer.getElapsedTimeInMilliSec();
		double homographyDuration = homographyEnd - homographyStart;
		cout << "homography : " << homographyDuration << endl;


		/*if(homographyFound){
		LOGE("refine homography found");
		}
		else{
		LOGE("refine homography not found");
		}*/

		// Get a result homography as result of matrix product of refined and rough homographies:
		if (homographyFound)
		{
			Mat mulHomography = m_roughHomography*m_refinedHomography;//don't know why but can't use info.homography = m_roughHomography * m_refinedHomography
			info.homography = mulHomography;

			m_estimatedHomography = info.homography;
			m_initialHomography = info.homography;


			initial.clear();
			before.clear();
			after.clear();
			patternPoints.clear();
			vector<Point2f> warpedPoints;
			vector<Point2f> queryPoints;
			for (int i = 0; i < m_matches.size(); i++)
			{
				warpedPoints.push_back(warpedKeypoints[m_matches[i].queryIdx].pt);
			}

			perspectiveTransform(warpedPoints, queryPoints, m_roughHomography);
			for (int i = 0; i < m_matches.size(); i++){
				//if(isMultiScale){
				//patternPoints.push_back(m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints[m_matches[i].trainIdx].pt);
				patternPoints.push_back(patternKeyPoints[m_matches[i].trainIdx].pt);
				//}else{
				//	patternPoints.push_back(m_pattern.keypoints[m_matches[i].trainIdx].pt);
				//}
				//initial.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
				//before.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
				//after.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);

				initial = queryPoints;
				before = queryPoints;
				after = queryPoints;
			}
			float err = arerror.computeError(*this, m_roughHomography);
			cout << "end compute error" << endl;


			//printMat("info.homography",info.homography);
			//printMat("rough homography", m_roughHomography);
			//printMat("refine homography",m_refinedHomography);
			//printMat("mul homography", mul);


			//LOGE("draw rough homography");
			perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
			info.draw2dContour(image, CV_RGB(0, 200, 0));

			// Transform contour with precise homography
			//LOGE("draw refine homography");
			perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
			//#if _DEBUG
			info.draw2dContour(image, CV_RGB(200, 0, 200));
			perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);


			cout << "begin draw contours" << endl;
			m_roughHomography = info.homography;
			drawing.drawContours(*this, image, info, indexes);
			cout << "end draw contours" << endl;
			//drawing.draw2Contours(*this, image, info, matchIndexes, estiIndexes);
			perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);


			m_lostFrameNum = 0;
			estimatedHomographyFound = true;
		}
		else
		{
			info.homography = m_roughHomography;
			m_lostFrameNum++;
			if (m_lostFrameNum >= maxLostFrames){
				estimatedHomographyFound = false;
			}
		}
		cout << "warped matches size: " << warpedMatches.size() << endl;
		cout << "begin handle tracked points outer" << endl;
		handleTrackedPoints(image, image);
		m_grayImgPrev = m_grayImg.clone();
		return homographyFound;
	}
	return homographyFound;
}

bool PatternDetector::simpleTracking(Mat& image, PatternTrackingInfo& info)
{
	cout << "begin simple tracking" << endl;
	bool homographyFound = false;
	cout << "estimated homography not found" << endl;
	// Extract feature points from input gray image
	Timer timer;
	timer.start();
	double extractStart = timer.getElapsedTimeInMilliSec();
	cout << "m_grayImage: " << m_grayImg.cols << "  " << m_grayImg.rows << endl;
	extractFeatures(m_grayImg, m_queryKeypoints, m_queryDescriptors);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	double extractDuration = extractEnd - extractStart;
	cout << "extract: " << extractDuration << endl;

	// Get matches with current pattern
	double matchStart = timer.getElapsedTimeInMilliSec();
	cout << "begin get matches" << endl;

	vector<int> indexes;
	indexes.clear();
	vector<int> estiIndexes;
	vector<int> matchIndexes;
	string matchstr;
	cout << "begin match keyframes" << endl;
	m_pattern.keyframeIndex = matchKeyFrames(m_estimatedHomography, indexes, matchIndexes, estiIndexes, matchstr);
	cout << "end match keyframes" << endl;
	cout << m_pattern.keyframeIndex << endl;
	string str = "index size: " + intToString(indexes.size());
	putText(image, matchstr, Point(10, 50), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 200, 0));
	if (indexes.size() == 0)
	{
		//return false;
	}
	if (m_pattern.keyframeIndex == -1)
	{
		m_pattern.keyframeIndex = 0;
	}
	m_pattern.keyframeIndex = -1;
	/*for (int i = 0; i < m_pattern.keyframeList.size(); i++)
	{
	indexes.push_back(i);
	}*/
	//indexes.push_back(4);
	//matchIndexes = indexes;
	//estiIndexes = indexes;
	cout << "get indexes" << endl;
	cout << "indexes size: " << indexes.size() << endl;
	for (int i = 0; i < indexes.size(); i++)
	{
		cout << indexes[i] << endl;
	}
	cout << "match indeses size: " << matchIndexes.size() << endl;
	for (int i = 0; i < matchIndexes.size(); i++)
	{
		cout << matchIndexes[i] << endl;
	}
	cout << "esti indexes size: " << estiIndexes.size() << endl;
	for (int i = 0; i < estiIndexes.size(); i++)
	{
		cout << estiIndexes[i] << endl;
	}
	//getMatches(m_queryDescriptors, m_pattern.keyframeIndex,m_matches);
	//use origi
	//indexes.clear();
	//indexes.push_back(0);
	getMatches(m_queryDescriptors, indexes, m_matches);
	cout << indexes.size() << endl;
	if (indexes.size() != 0)
	{
		nowMatchedKeyframes = indexes;
	}
	cout << m_queryDescriptors.size() << "  " << m_pattern.keyframeList[indexes[0]].descriptors.size() << endl;
	cout << "before homo size: " << m_matches.size() << endl;
	cout << "end get matches: " << m_matches.size() << endl;
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;
	//cout<<m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size()<<endl;
	//cout<<m_pattern.keypoints.size()<<endl;

	vector<KeyPoint> patternKeyPoints;
	patternKeyPoints.clear();
	for (int i = 0; i < indexes.size(); i++)
	{
		patternKeyPoints.insert(patternKeyPoints.end(), m_pattern.keyframeList[indexes[i]].keypoints.begin(), m_pattern.keyframeList[indexes[i]].keypoints.end());
	}

	// Find homography transformation and detect good matches
	double homographyStart = timer.getElapsedTimeInMilliSec();
	//if(isMultiScale){
	homographyFound = refineMatchesWithHomography(
		m_queryKeypoints,
		//m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints,
		patternKeyPoints,
		homographyReprojectionThreshold,
		m_matches,
		m_roughHomography);
	cout << "homography found is: " << homographyFound << "  " << m_matches.size() << endl;
	cout << m_queryKeypoints.size() << "  " << m_pattern.keyframeList[indexes[0]].keypoints.size() << endl;
	cout << "after homo size: " << m_matches.size() << endl;
	//}
	//else
	//{
	//	homographyFound = refineMatchesWithHomography(
	//			m_queryKeypoints,
	//			m_pattern.keypoints,
	//			homographyReprojectionThreshold,
	//			m_matches,
	//			m_roughHomography);
	//}
	double homographyEnd = timer.getElapsedTimeInMilliSec();
	double homographyDuration = homographyEnd - homographyStart;
	cout << "homography: " << homographyDuration << endl;


	// Transform contour with rough homography
	if (homographyFound){

		info.homography = m_roughHomography;

		m_estimatedHomography = info.homography;
		m_initialHomography = info.homography;
		cout << "begin compute error" << endl;

		cout << "matches size: " << m_matches.size() << endl;
		initial.clear();
		before.clear();
		after.clear();
		patternPoints.clear();
		for (int i = 0; i < m_matches.size(); i++){
			//if(isMultiScale){
			//patternPoints.push_back(m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints[m_matches[i].trainIdx].pt);
			patternPoints.push_back(patternKeyPoints[m_matches[i].trainIdx].pt);
			//}else{
			//	patternPoints.push_back(m_pattern.keypoints[m_matches[i].trainIdx].pt);
			//}
			initial.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
			before.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
			after.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
		}
		float err = arerror.computeError(*this, m_roughHomography);
		cout << "end compute error" << endl;
		//if(isMultiScale)
		{
			/*perspectiveTransform(m_pattern.keyframeList[m_pattern.keyframeIndex].points2d, info.points2d, m_roughHomography);
			info.draw2dContour(image, CV_RGB(200,0,0));
			cout<<"keyframe size: "<<m_pattern.keyframeList.size()<<endl;
			for(int i=0;i<m_pattern.keyframeList.size();i++){
			perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, m_roughHomography);
			info.draw2dContour(image, CV_RGB(200,200,200));

			}*/
			//drawContours(image, info);

			cout << "begin draw contours" << endl;
			drawing.drawContours(*this,image, info, indexes);
			cout << "end draw contours" << endl;
			//drawing.draw2Contours(*this, image, info, matchIndexes, estiIndexes);
			perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);

		}
		//else
		//{
		//	perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
		//}
		perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
		//info.draw2dContour(image, CV_RGB(200,0,0));
		m_lostFrameNum = 0;
		estimatedHomographyFound = true;
	}
	else
	{
		m_lostFrameNum++;
		if (m_lostFrameNum >= maxLostFrames){
			estimatedHomographyFound = false;
		}
	}
	
	
	//cout << "err is: " << err << endl;

	cout << "begin handle tracked points outer" << endl;
	handleTrackedPoints(image, image);
	m_grayImgPrev = m_grayImg.clone();
	return homographyFound;
}

bool PatternDetector::simpleTrackingNew(Mat& image, PatternTrackingInfo& info)
{
	cout << "begin simple tracking" << endl;
	bool homographyFound = false;
	cout << "estimated homography not found" << endl;
	// Extract feature points from input gray image
	Timer timer;
	timer.start();
	double extractStart = timer.getElapsedTimeInMilliSec();
	cout << "m_grayImage: " << m_grayImg.cols << "  " << m_grayImg.rows << endl;
	extractFeatures(m_grayImg, m_queryKeypoints, m_queryDescriptors);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	double extractDuration = extractEnd - extractStart;
	cout << "extract: " << extractDuration << endl;

	// Get matches with current pattern
	double matchStart = timer.getElapsedTimeInMilliSec();
	cout << "begin get matches" << endl;

	vector<int> indexes;
	indexes.clear();
	vector<int> estiIndexes;
	vector<int> matchIndexes;
	string matchstr;
	cout << "begin match keyframes" << endl;
	m_pattern.keyframeIndex = matchKeyFramesNew(m_estimatedHomography, indexes, matchIndexes, estiIndexes, matchstr);
	cout << "end match keyframes" << endl;
	cout << m_pattern.keyframeIndex << endl;
	string str = "index size: " + intToString(indexes.size());
	putText(image, matchstr, Point(10, 50), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 200, 0));
	if (indexes.size() == 0)
	{
		//return false;
	}
	if (m_pattern.keyframeIndex == -1)
	{
		m_pattern.keyframeIndex = 0;
	}
	m_pattern.keyframeIndex = -1;
	/*for (int i = 0; i < m_pattern.keyframeList.size(); i++)
	{
	indexes.push_back(i);
	}*/
	//indexes.push_back(4);
	//matchIndexes = indexes;
	//estiIndexes = indexes;
	cout << "get indexes" << endl;
	cout << "indexes size: " << indexes.size() << endl;
	for (int i = 0; i < indexes.size(); i++)
	{
		cout << indexes[i] << endl;
	}
	cout << "match indeses size: " << matchIndexes.size() << endl;
	for (int i = 0; i < matchIndexes.size(); i++)
	{
		cout << matchIndexes[i] << endl;
	}
	cout << "esti indexes size: " << estiIndexes.size() << endl;
	for (int i = 0; i < estiIndexes.size(); i++)
	{
		cout << estiIndexes[i] << endl;
	}
	//getMatches(m_queryDescriptors, m_pattern.keyframeIndex,m_matches);
	//use origi
	//indexes.clear();
	//indexes.push_back(0);
	getMatches(m_queryDescriptors, indexes, m_matches);
	cout << indexes.size() << endl;
	if (indexes.size() != 0)
	{
		nowMatchedKeyframes = indexes;
	}
	cout << m_queryDescriptors.size() << "  " << m_pattern.keyframeList[indexes[0]].descriptors.size() << endl;
	cout << "before homo size: " << m_matches.size() << endl;
	cout << "end get matches: " << m_matches.size() << endl;
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;
	//cout<<m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size()<<endl;
	//cout<<m_pattern.keypoints.size()<<endl;

	vector<KeyPoint> patternKeyPoints;
	patternKeyPoints.clear();
	for (int i = 0; i < indexes.size(); i++)
	{
		patternKeyPoints.insert(patternKeyPoints.end(), m_pattern.keyframeList[indexes[i]].keypoints.begin(), m_pattern.keyframeList[indexes[i]].keypoints.end());
	}

	// Find homography transformation and detect good matches
	double homographyStart = timer.getElapsedTimeInMilliSec();
	//if(isMultiScale){
	homographyFound = refineMatchesWithHomography(
		m_queryKeypoints,
		//m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints,
		patternKeyPoints,
		homographyReprojectionThreshold,
		m_matches,
		m_roughHomography);
	cout << "homography found is: " << homographyFound << "  " << m_matches.size() << endl;
	cout << m_queryKeypoints.size() << "  " << m_pattern.keyframeList[indexes[0]].keypoints.size() << endl;
	cout << "after homo size: " << m_matches.size() << endl;
	//}
	//else
	//{
	//	homographyFound = refineMatchesWithHomography(
	//			m_queryKeypoints,
	//			m_pattern.keypoints,
	//			homographyReprojectionThreshold,
	//			m_matches,
	//			m_roughHomography);
	//}
	double homographyEnd = timer.getElapsedTimeInMilliSec();
	double homographyDuration = homographyEnd - homographyStart;
	cout << "homography: " << homographyDuration << endl;


	// Transform contour with rough homography
	if (homographyFound){

		info.homography = m_roughHomography;

		m_estimatedHomography = info.homography;
		m_initialHomography = info.homography;
		cout << "begin compute error" << endl;

		cout << "matches size: " << m_matches.size() << endl;
		initial.clear();
		before.clear();
		after.clear();
		patternPoints.clear();
		for (int i = 0; i < m_matches.size(); i++){
			//if(isMultiScale){
			//patternPoints.push_back(m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints[m_matches[i].trainIdx].pt);
			patternPoints.push_back(patternKeyPoints[m_matches[i].trainIdx].pt);
			//}else{
			//	patternPoints.push_back(m_pattern.keypoints[m_matches[i].trainIdx].pt);
			//}
			initial.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
			before.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
			after.push_back(m_queryKeypoints[m_matches[i].queryIdx].pt);
		}
		float err = arerror.computeError(*this, m_roughHomography);
		cout << "end compute error" << endl;
		//if(isMultiScale)
		{
			/*perspectiveTransform(m_pattern.keyframeList[m_pattern.keyframeIndex].points2d, info.points2d, m_roughHomography);
			info.draw2dContour(image, CV_RGB(200,0,0));
			cout<<"keyframe size: "<<m_pattern.keyframeList.size()<<endl;
			for(int i=0;i<m_pattern.keyframeList.size();i++){
			perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, m_roughHomography);
			info.draw2dContour(image, CV_RGB(200,200,200));

			}*/
			//drawContours(image, info);

			cout << "begin draw contours" << endl;
			drawing.drawContours(*this, image, info, indexes);
			cout << "end draw contours" << endl;
			//drawing.draw2Contours(*this, image, info, matchIndexes, estiIndexes);
			perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);

		}

		//match keyframes
		vector<int> indexes;
		vector<int> matchindexes;
		vector<int> estiindexes;
		string str;
		matchKeyframesWithPolygon(m_roughHomography, indexes);
		//matchKeyFrames(m_roughHomography, indexes,matchindexes,estiindexes,str);
		nowMatchedKeyframes = indexes;
		//else
		//{
		//	perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
		//}
		perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
		//info.draw2dContour(image, CV_RGB(200,0,0));
		m_lostFrameNum = 0;
		estimatedHomographyFound = true;
	}
	else
	{
		m_lostFrameNum++;
		if (m_lostFrameNum >= maxLostFrames){
			estimatedHomographyFound = false;
		}
	}


	//cout << "err is: " << err << endl;

	cout << "begin handle tracked points outer" << endl;
	handleTrackedPoints(image, image);
	m_grayImgPrev = m_grayImg.clone();
	return homographyFound;
}

bool PatternDetector::correctDistance()
{
	if (nowDistance > 0.5f&&nowDistance < 2.5f)
	{
		return true;
	}
	return false;
}

bool PatternDetector::findPattern(Mat& image, PatternTrackingInfo& info)
{
	// Convert input image to gray
	Timer timer;
	timer.start();
	double getGrayStart = timer.getElapsedTimeInMilliSec();
	getGray(image, m_grayImg);
	double getGrayEnd = timer.getElapsedTimeInMilliSec();
	double getGrayDuration = getGrayEnd - getGrayStart;
	cout << "get gray: " << getGrayDuration << endl;


	//int maxLostFrames = 5;
	m_roughHomography = info.homography;
	bool homographyFound = false;
	//LOGE("before get branch cols:%d, rows:%d",info.homography.cols,info.homography.rows);
	//homographyFound=false;
	cout << m_queryKeypoints.size() << endl;
	if (!needNewPoints() && enableOpticalFlow&&correctDistance()){
		homographyFound = OpticalTracking(image, info);
		
	}
	else
	{
		if (estimatedHomographyFound && enableWrap&&correctDistance())
		{
			cout << "begin warped tracking" << endl;
			homographyFound = warpedTrackingNew(image, info);
			
		}
		else
		{
			homographyFound = simpleTrackingNew(image, info);
			
		}
		m_opticalFrameNum = 0;
	}

	cout << "end find pattern homography found: " << homographyFound << endl;
	stringstream disstream;
	if (eyes.size() > 0)
	{

		disstream << eyes.back().distance;
		//disstream<<
		string disstring = disstream.str();
		drawing.drawText(*this, image, disstring, Point(10, 70), Scalar(200, 0, 0));
		nowDistance = eyes.back().distance;
	}
	return homographyFound;
}


void PatternDetector::getGray(const Mat& image, Mat& gray)
{
	if (image.channels() == 3)
		cvtColor(image, gray, CV_BGR2GRAY);
	else if (image.channels() == 4)
		cvtColor(image, gray, CV_BGRA2GRAY);
	else if (image.channels() == 1)
		gray = image;
}

bool PatternDetector::extractFeatures(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors) const
{
	assert(!image.empty());
	//assert(image.channels() == 1);

	Timer timer;
	timer.start();
	double detectStart = timer.getElapsedTimeInMilliSec();
	m_detector->detect(image, keypoints);
	double detectEnd = timer.getElapsedTimeInMilliSec();
	double detectDuration = detectEnd - detectStart;
	cout << "detect time: " << detectDuration << endl;

	cout << "keypoints size: " << keypoints.size() << endl;
	if (keypoints.empty())
		return false;

	double retainStart = timer.getElapsedTimeInMilliSec();
	int num = 200;
	if (keypoints.size() > num)
	{
		KeyPointsFilter filter;
		filter.retainBest(keypoints, num);
		//keyframe.keyPoints.resize(num);
	}
	double retainEnd = timer.getElapsedTimeInMilliSec();
	double retainDuration = retainEnd - retainStart;
	cout << "retain best: " << retainDuration << endl;


	double extractStart = timer.getElapsedTimeInMilliSec();
	m_extractor->compute(image, keypoints, descriptors);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	cout << "descriptor size: " << descriptors.size() << endl;
	if (keypoints.empty())
		return false;

	return true;
}

void PatternDetector::getMatches(const Mat& queryDescriptors, vector<DMatch>& matches)
{
	matches.clear();

	if (enableRatioTest)
	{
		// To avoid NaN's when best match has zero distance we will use inversed ratio.
		const float minRatio = 1.f / 1.5f;

		// KNN match will return 2 nearest matches for each query descriptor
		m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);

		for (size_t i = 0; i < m_knnMatches.size(); i++)
		{
			const DMatch& bestMatch = m_knnMatches[i][0];
			const DMatch& betterMatch = m_knnMatches[i][1];

			float distanceRatio = bestMatch.distance / betterMatch.distance;

			// Pass only matches where distance ratio between
			// nearest matches is greater than 1.5 (distinct criteria)
			if (distanceRatio < minRatio)
			{
				matches.push_back(bestMatch);
			}
		}
	}
	else
	{
		//if(isMultiScale)
		{
			cout << "multi match start" << endl;
			m_matcher->match(queryDescriptors, m_pattern.keyframeList[m_pattern.keyframeIndex].descriptors, matches);
			cout << "match size: " << matches.size() << endl;
			cout << "multi match end" << endl;
		}
		//else
		//{
		//	cout<<"non multi match start"<<endl;
		//	// Perform regular match
		//	Timer timer;
		//	timer.start();
		//	double regularMatchStart=timer.getElapsedTimeInMilliSec();
		//	m_matcher->match(queryDescriptors, m_pattern.descriptors, matches);
		//	double regularMatchEnd=timer.getElapsedTimeInMilliSec();
		//	double regularMatchDuration=regularMatchEnd-regularMatchStart;
		//	cout<<"regular match: "<<regularMatchDuration<<endl;
		//	cout<<"non multi match send"<<endl;
		//}
	}
}


void PatternDetector::getMatches(const Mat& queryDescriptors, int index, vector<DMatch>& matches)
{
	matches.clear();

	if (enableRatioTest)
	{
		// To avoid NaN's when best match has zero distance we will use inversed ratio.
		const float minRatio = 1.f / 1.5f;

		// KNN match will return 2 nearest matches for each query descriptor
		m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);

		for (size_t i = 0; i < m_knnMatches.size(); i++)
		{
			const DMatch& bestMatch = m_knnMatches[i][0];
			const DMatch& betterMatch = m_knnMatches[i][1];

			float distanceRatio = bestMatch.distance / betterMatch.distance;

			// Pass only matches where distance ratio between
			// nearest matches is greater than 1.5 (distinct criteria)
			if (distanceRatio < minRatio)
			{
				matches.push_back(bestMatch);
			}
		}
	}
	else
	{
		//if(isMultiScale)
		{
			m_matcher->match(queryDescriptors, m_pattern.keyframeList[index].descriptors, matches);
		}
		//else
		//{
		//	// Perform regular match
		//	Timer timer;
		//	timer.start();
		//	double regularMatchStart=timer.getElapsedTimeInMilliSec();
		//	m_matcher->match(queryDescriptors, m_pattern.descriptors, matches);
		//	double regularMatchEnd=timer.getElapsedTimeInMilliSec();
		//	double regularMatchDuration=regularMatchEnd-regularMatchStart;
		//	cout<<"regular match: "<<regularMatchDuration<<endl;
		//}
	}
}

void PatternDetector::getMatches(const Mat& queryDescriptors, vector<int>& indexes, vector<DMatch>& matches)
{
	cout << "begin matches with indexes" << endl;
	matches.clear();

	if (enableRatioTest && false)
	{
		// To avoid NaN's when best match has zero distance we will use inversed ratio.
		const float minRatio = 1.f / 1.5f;

		// KNN match will return 2 nearest matches for each query descriptor
		m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);

		for (size_t i = 0; i < m_knnMatches.size(); i++)
		{
			const DMatch& bestMatch = m_knnMatches[i][0];
			const DMatch& betterMatch = m_knnMatches[i][1];

			float distanceRatio = bestMatch.distance / betterMatch.distance;

			// Pass only matches where distance ratio between
			// nearest matches is greater than 1.5 (distinct criteria)
			if (distanceRatio < minRatio)
			{
				matches.push_back(bestMatch);
			}
		}
	}
	else
	{
		//if(isMultiScale)
		{
			if (indexes.size() == 0)//match failed,push all keyframes into indexes to implement a full match
			{
				int size = m_pattern.keyframeList.size();
				for (int i = 0; i < size; i++)
				{
					indexes.push_back(i);
				}
			}
			cout << "begin calc rows" << endl;
			/*int cols = m_pattern.descriptors.cols;
			int type = m_pattern.descriptors.type();
			int rows = 0;
			for (int i = 0; i < indexes.size(); i++)
			{
			rows += m_pattern.keyframeList[i].descriptors.rows;
			}*/
			cout << "begin combine descriptors" << endl;
			Mat descriptors;
			for (int i = 0; i < indexes.size(); i++)
			{
				cout << m_pattern.keyframeList.size() << "  " << indexes[i] << endl;
				Mat& kfDescriptors = m_pattern.keyframeList[indexes[i]].descriptors;
				cout << "keyframe: " << i << endl;
				cout << kfDescriptors.size() << endl;
				descriptors.push_back(kfDescriptors);
			}
			cout << "end combine descriptors" << endl;
			cout << descriptors.size() << endl;
			m_matcher->match(queryDescriptors, descriptors, matches);
		}
		//else
		//{
		//	// Perform regular match
		//	Timer timer;
		//	timer.start();
		//	double regularMatchStart=timer.getElapsedTimeInMilliSec();
		//	m_matcher->match(queryDescriptors, m_pattern.descriptors, matches);
		//	double regularMatchEnd=timer.getElapsedTimeInMilliSec();
		//	double regularMatchDuration=regularMatchEnd-regularMatchStart;
		//	cout<<"regular match: "<<regularMatchDuration<<endl;
		//}
	}
	cout << "end matches with indexes" << endl;
}

bool PatternDetector::refineMatchesWithHomography
(
const vector<KeyPoint>& queryKeypoints,
const vector<KeyPoint>& trainKeypoints,
float reprojectionThreshold,
vector<DMatch>& matches,
Mat& homography
)
{
	cout << "begin refine matches with homography" << endl;
	cout << "begin matches size: " << matches.size() << endl;
	const int minNumberMatchesAllowed = 8;

	if (matches.size() < minNumberMatchesAllowed)
	{
		return false;
	}

	// Prepare data for findHomography
	vector<Point2f> srcPoints(matches.size());
	vector<Point2f> dstPoints(matches.size());
	cout << "begin copy" << endl;
	for (size_t i = 0; i < matches.size(); i++)
	{
		srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
		dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
	}

	// Find homography matrix and get inliers mask
	vector<unsigned char> inliersMask(srcPoints.size());
	Timer timer;
	timer.start();
	cout << "timer start" << endl;
	double findHomographyStart = timer.getElapsedTimeInMilliSec();
	cout << "begin homography" << endl;
	homography = findHomography(srcPoints,
		dstPoints,
		CV_FM_RANSAC,
		reprojectionThreshold,
		inliersMask);
	cout << "end homography" << endl;

	double findHomographyEnd = timer.getElapsedTimeInMilliSec();
	double findHomographyDuration = findHomographyEnd - findHomographyStart;
	cout << "find homography duration: " << findHomographyDuration << endl;

	vector<DMatch> inliers;
	for (size_t i = 0; i<inliersMask.size(); i++)
	{
		if (inliersMask[i])
			inliers.push_back(matches[i]);
	}

	matches.swap(inliers);
	cout << "matches size: " << matches.size() << endl;
	cout << "end refine matches with homography" << endl;
	return matches.size() > minNumberMatchesAllowed;
}

// determine which tracked point should be accepted
bool PatternDetector::acceptTrackedPoint(int i) 
{
	return status[i];
}



// handle the currently tracked points
void PatternDetector::handleTrackedPoints(Mat &frame, Mat &output, Scalar scalar) {
	cout << "begin handle tracked points inner" << endl;
	if (isShowPoints == false)
	{
		return;
	}
	// for all tracked points
	for (int i = 0; i < after.size(); i++) {
		// draw line and circle
		circle(output, after[i], 3, scalar, -1);
	}


}

