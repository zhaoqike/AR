

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
	conprint << name << endl;
	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			conprint << mat.at<double>(i, j) << ' ';
		}
		conprint << endl;
	}
	conprint << endl;
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


Point3f PatternDetector::point2dTo3d(Size oriSize, Point2f p2d)
{
	// Image dimensions
	const float w = oriSize.width;
	const float h = oriSize.height;

	// Normalized dimensions:
	const float maxSize = (max)(w, h);
	const float unitW = w / maxSize;
	const float unitH = h / maxSize;

	float x = -unitW + 2 * unitW*p2d.x / w;
	float y = -unitH + 2 * unitH*p2d.y / h;
	float z = 0;
	return Point3f(x, y, x);
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

	keyframe.center = point2dTo3d(oriSize, Point2f((ltx + rtx) / 2, (lty + lby) / 2));

	double scalew = (double)screenWidth / (double)keyframe.frame.cols;
	double scaleh = (double)screenHeight / (double)keyframe.frame.rows;
	//double scale=(scalew+scaleh)/2;
	double scale = getWindowDivPictureScale(keyframe.frame.cols, keyframe.frame.rows);

	//double scale = 1.0;

	conprint << "scalew scaleh scale: " << scalew << "  " << scaleh << "  " << scale << endl;


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
	int layerNum = 0;
	if (isMultiScale)
	{

		double scale = getPictureDivWindowScale(width, height);
		if (false){
			int level = 1;
			double dlevel = log(scale) / log(2);
			if (dlevel < 1)
			{
				level = 1;
			}
			else
			{
				dlevel = ceil(dlevel);
			}
			layerNum = level;
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
			layerNum = 2 * level+1;
		}
	}
	else
	{
		layerNum = 1;
	}
	conprint << "layerNum: " << layerNum << endl;
	if (layerNum > indexCount)
	{
		//indexCount = layerNum;
	}
	return layerNum;
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

	conprint << "start end" << endl;
	//conprint << startx << "  " << endx << "  " << starty << "  " << endy << endl;
	conprint << "scale and num: " << scale << "  " << num << endl;
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
		conprint << "range" << endl;
		conprint << xRange << "  " << yRange << endl;
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
					//conprint << "layer: " << level << "  " << i << "  " << j << " xleft" << endl;
					//conprint << kf.rect.x << endl;
					rect.x = 0;
				}
				if (rect.y < 0)
				{
					//conprint << "layer: " << level << "  " << i << "  " << j << " ytop" << endl;
					//conprint << kf.rect.y << endl;
					rect.y = 0;
				}
				if (rect.x + rect.width>oriWidth)
				{
					//conprint << "layer: " << level << "  " << i << "  " << j << " xright" << endl;
					//conprint << kf.rect.x + kf.rect.width << "  " << oriWidth << endl;
					rect.x -= rect.x + rect.width - oriWidth;
				}
				if (rect.y + rect.height > oriHeight)
				{
					//conprint << "layer: " << level << "  " << i << "  " << j << " ybottom" << endl;
					//conprint << kf.rect.y + kf.rect.height << "  " << oriHeight << endl;
					rect.y -= rect.y + rect.height - oriHeight;
				}
				rectList.push_back(rect);
			}
		}
	}
	cout<<"rect list size " <<rectList.size()<<endl;
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
	conprint << "=====================================" << endl;
	layer.keyframeList.clear();
	int level = lev;
	conprint << "layer: " << lev << " start" << endl;
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

	conprint << "start end" << endl;
	//conprint << startx << "  " << endx << "  " << starty << "  " << endy << endl;
	conprint << "scale and num: " << scale << "  " << num << endl;
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
		conprint << "range" << endl;
		conprint << xRange << "  " << yRange << endl;
		for (int i = 0; i < num; i++)
		{
			xList.push_back(i*xRange);
			yList.push_back(i*yRange);
		}
		/*for (int i = 0; i < num; i++)
		{
		conprint << "list " << i << endl;
		conprint << xList[i] << "  " << yList[i] << endl;
		}*/
		for (int i = 0; i < num; i++)
		{
			for (int j = 0; j < num; j++)
			{
				KeyFrame kf;
				kf.rect = Rect(xList[i], yList[j], width, height);
				//conprint << xList[i] - width / 2 << "  " << yList[j] - height / 2 << "  " << width << "  " << height << endl;
				//conprint << kf.rect.x << "  " << kf.rect.y << "   " << kf.rect.x + kf.rect.width << "  " << kf.rect.y + kf.rect.height << endl;
				if (kf.rect.x < 0)
				{
					//conprint << "layer: " << level << "  " << i << "  " << j << " xleft" << endl;
					//conprint << kf.rect.x << endl;
					kf.rect.x = 0;
				}
				if (kf.rect.y < 0)
				{
					//conprint << "layer: " << level << "  " << i << "  " << j << " ytop" << endl;
					//conprint << kf.rect.y << endl;
					kf.rect.y = 0;
				}
				if (kf.rect.x + kf.rect.width>oriWidth)
				{
					//conprint << "layer: " << level << "  " << i << "  " << j << " xright" << endl;
					//conprint << kf.rect.x + kf.rect.width << "  " << oriWidth << endl;
					kf.rect.x -= kf.rect.x + kf.rect.width - oriWidth;
				}
				if (kf.rect.y + kf.rect.height > oriHeight)
				{
					//conprint << "layer: " << level << "  " << i << "  " << j << " ybottom" << endl;
					//conprint << kf.rect.y + kf.rect.height << "  " << oriHeight << endl;
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
	conprint << "layer: " << lev << " end" << endl;
	conprint << "+++++++++++++++++++++++++++++++++++++++++" << endl;

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
	conprint << "keyframe information" << endl;
	for (int i = 0; i < keyFrameList.size(); i++)
	{
		conprint << "keyframe: " << i << endl;
		conprint << keyFrameList[i].keypoints.size() << "  " << keyFrameList[i].descriptors.size() << endl;
	}

	double scale = getPictureDivWindowScale(img.cols, img.rows);
	int level = log(scale) / log(2);
	vector<Rect> rectList1;
	cutImage(ori, level, rectList1);
	conprint << "rect list : " << rectList1.size() << endl;
	for (int i = 0; i < rectList1.size(); i++)
	{
		conprint << rectList1[i].x << "  " << rectList1[i].y << "  " << rectList1[i].width << "  " << rectList1[i].height << endl;
	}


}

void PatternDetector::makeLayerList(const Mat& image, vector<Layer>& layerList, int layers)
{
	layerList.resize(layers);
	cout<<"layer num: "<<layers<<endl;
	for (int i = 0; i < layerList.size(); i++)
	{
		conprint << "make layer list start: " << i << endl;
		makeKeyFrameList(image, layerList[i], i);
		cout<<layerList[i].keyframeList.size()<<endl;
		conprint << "make layer list end: " << i << endl;
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
	cout<<"build"<<endl;
	//if(isMultiScale)
	{
		bool useNewScale = true;
#ifdef WIN32
		useNewScale = false;
#endif
		if (useNewScale)
		{
			int layerNum = getLayerNum(image.cols, image.rows);
			conprint << "layer num: " << layerNum << endl;
			makeLayerList(image, pattern.layerList, layerNum);

			//to put all keyframes in a vector
			pattern.keyframeList.clear();
			cout<<"after clear"<<endl;
			cout<<pattern.keyframeList.size()<<endl;
			for (int i = 0; i < pattern.layerList.size(); i++)
			{
				Layer& layer = pattern.layerList[i];
				for (int j = 0; j < layer.keyframeList.size(); j++)
				{
					pattern.keyframeList.push_back(layer.keyframeList[j]);
				}
			}
			cout<<"after wwwclear"<<endl;
			cout<<pattern.keyframeList.size()<<endl;
			if (isMultiScale == false)
			{
				conprint << "keyframe list size: " << pattern.keyframeList.size() << endl;
				if (pattern.keyframeList.size() == 1)
				{
					//conprint << "keyframe list size: " << pattern.keyframeList.size() << endl;
					conprint << "success" << endl;
				}
				else
				{
					conprint << "error" << endl;
				}
			}

			for (int i = 0; i < pattern.layerList.size(); i++)
			{
				conprint << "layer: " << i << " has " << pattern.layerList[i].keyframeList.size() << " keyframes";
			}
			conprint << "all keyframe num is: " << pattern.keyframeList.size() << endl;
			cout<<"num: "<<endl;
			cout<<pattern.keyframeList.size()<<endl;


			for (int i = 0; i < pattern.layerList.size(); i++)
			{
				Mat layerImg = image.clone();
				//conprint << "layer: " << i << endl;
				Layer& layer = pattern.layerList[i];
				for (int j = 0; j < layer.keyframeList.size(); j++)
				{
					//conprint << "keyframe: " << j << endl;
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
					//conprint << "keyframe: " << j << endl;
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
		conprint << "init models: " << kfmodels.size() << endl;
		conprint << pattern.keyframeList.size() << endl;
		conprint << kfmodels.size() << endl;
		int meshNum = pointList.size();
		conprint << meshNum << endl;
		for (int i = 0; i<pointList.size(); i++)
		{
			int pathIndex=i%modelPathList.size();

			Model model(modelPathList[pathIndex]);
			model.p3d = point2dTo3d(image.size(), pointList[i]);

			//kfmodels[i].pmesh=makeMesh(modelPathList[ pathIndex]);
			kfmodels.push_back(model);
		}
		conprint << kfmodels.size() << endl;
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



bool PatternDetector::findPatternFirstStage(Mat& image, PatternTrackingInfo& info)
{
	conprint << "find pattern first stage" << endl;
	Timer timer;
	timer.start();
	double getGrayStart = timer.getElapsedTimeInMilliSec();
	getGray(image, m_grayImg);
	double getGrayEnd = timer.getElapsedTimeInMilliSec();
	double getGrayDuration = getGrayEnd - getGrayStart;
	conprint << "get gray: " << getGrayDuration << endl;


	int maxLostFrames = 5;
	m_lastHomography = info.homography;
	bool homographyFound = false;
	//LOGE("before get branch cols:%d, rows:%d",info.homography.cols,info.homography.rows);
	//homographyFound=false;
	/*if (estimatedHomographyFound && enableWrap)
	{

	LOGE("homography found first");
	//LOGE("type:%d, rows:%d, cold:%d",m_lastHomography.type(),m_lastHomography.rows,m_lastHomography.cols);
	// Warp image using found homography
	double warpStart=timer.getElapsedTimeInMilliSec();
	warpPerspective(m_grayImg, m_firstStageImg, m_lastHomography, m_pattern.size, WARP_INVERSE_MAP );
	//warpPerspective(m_grayImg, m_warpedImg, m_lastHomography, m_pattern.size, WARP_INVERSE_MAP | INTER_CUBIC);
	double warpEnd=timer.getElapsedTimeInMilliSec();
	double warpDuration=warpEnd-warpStart;
	conprint<<"warp image: "<<warpDuration<<endl;
	}
	else*/
	{
		conprint << "homography not found first" << endl;
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
	conprint << "find pattern second stage" << endl;
	Timer timer;
	timer.start();

	m_firstToSecondMutex.lock();
	m_secondStageImg = m_firstToSecondImg;
	m_firstToSecondMutex.unlock();

	int maxLostFrames = 5;
	m_lastHomography = info.homography;
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
	conprint<<"extract feature: "<<extractDuration<<endl;

	// Match with pattern
	double matchStart=timer.getElapsedTimeInMilliSec();
	getMatches(m_queryDescriptors, m_matches);
	double matchEnd=timer.getElapsedTimeInMilliSec();
	double matchDuration=matchEnd-matchStart;
	conprint<<"match feature: "<<matchDuration<<endl;



	}
	else*/
	{
		conprint << "homography not found second" << endl;
		// Extract feature points from input gray image
		Timer timer;
		timer.start();
		double extractStart = timer.getElapsedTimeInMilliSec();
		extractFeatures(m_secondStageImg, m_queryKeypoints, m_queryDescriptors);
		double extractEnd = timer.getElapsedTimeInMilliSec();
		double extractDuration = extractEnd - extractStart;
		conprint << "extract: " << extractDuration << endl;

		// Get matches with current pattern
		double matchStart = timer.getElapsedTimeInMilliSec();
		getMatches(m_queryDescriptors, m_matches);
		conprint << "match size: " << m_matches.size() << endl;
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
	conprint << "find pattern second stage" << endl;
	Timer timer;
	timer.start();



	int maxLostFrames = 5;
	m_lastHomography = info.homography;
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
	conprint<<"homography : "<<homographyDuration<<endl;



	// Get a result homography as result of matrix product of refined and rough homographies:
	if(homographyFound)
	{

	Mat mulHomography=m_lastHomography*m_refinedHomography;//don't know why but can't use info.homography = m_lastHomography * m_refinedHomography
	info.homography = mulHomography;


	//printMat("info.homography",info.homography);
	//printMat("rough homography", m_lastHomography);
	//printMat("refine homography",m_refinedHomography);
	//printMat("mul homography", mul);


	//LOGE("draw rough homography");
	perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);
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
	info.homography=m_lastHomography;
	m_lostFrameNum++;
	if(m_lostFrameNum>=maxLostFrames){
	estimatedHomographyFound=false;
	}
	}


	m_estimatedHomography=info.homography;

	}
	else*/
	{
		conprint << "homography not found second";
		conprint << "homography not found" << endl;
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
			m_lastHomography);
		double homographyEnd = timer.getElapsedTimeInMilliSec();
		double homographyDuration = homographyEnd - homographyStart;
		conprint << "homography: " << homographyDuration << endl;

		conprint << "homography found: " << homographyFound << endl;
		// Transform contour with rough homography
		if (homographyFound){

			info.homography = m_lastHomography;

			m_estimatedHomography = info.homography;



			perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);
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
	conprint << "in calc window area: " << endl;
	conprint << screenArea << endl;
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
	conprint << "screea area" << endl;
	conprint << screenArea << endl;
	return screenPoly;
}

bool PatternDetector::matchKeyframesWithPolygon(Mat& homography, vector<int>& indexes)
{
	indexes.clear();
	string str;
	if (isMultiScale)
	{

		indexes.clear();

		if (estimatedHomographyFound)
		{

			Timer timer;
			timer.start();
			conprint << "begin calc areas" << endl;
			MyPoint lt(0, 0), lb(0, screenHeight), rb(screenWidth, screenHeight), rt(screenWidth, 0);
			vector<MyPoint> screenPointList;
			screenPointList.push_back(lt);
			screenPointList.push_back(lb);
			screenPointList.push_back(rb);
			screenPointList.push_back(rt);
			MyPolygon screenPoly(screenPointList);

			double areas = timer.getElapsedTimeInMilliSec();
			double screenArea = screenHeight*screenWidth;// screenPoly.area_of_polygon(screenPoly.pointList);
			double areae = timer.getElapsedTimeInMilliSec();
			double aread = areae - areas;
			//conprint << "screen area time: " << aread << endl;
			//conprint << "screea area" << endl;
			//conprint << screenArea << endl;
			vector<AlphaState> alphaList;
			bool success = true;

			//test
			vector<MyPoint> patternPointList;
			PatternTrackingInfo info;
			info.homography = homography;
			perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
			for (int j = 0; j < 4; j++)
			{
				MyPoint p(info.points2d[j].x, info.points2d[j].y);
				patternPointList.push_back(p);
			}
			MyPolygon patternPoly(patternPointList);
			if (!patternPoly.issimple() || !patternPoly.isconvex())
			{
				success = false;
				return 0;
			}

			double polyStart = timer.getElapsedTimeInMilliSec();
			conprint << "begin calc all" << endl;
			for (int i = 0; i < m_pattern.keyframeList.size(); i++)
			{
				double ls = timer.getElapsedTimeInMilliSec();
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
				/*if (!keyframePoly.issimple() || !keyframePoly.isconvex())
				{
				success = false;
				break;
				}*/
				MyPolygon interPoly;
				double intes = timer.getElapsedTimeInMilliSec();
				bool success = screenPoly.intersect(screenPoly, keyframePoly, interPoly);
				double intee = timer.getElapsedTimeInMilliSec();
				double inted = intee - intes;
				double l1 = intee - ls;
				//conprint << "inter time: " << inted << endl;
				//conprint << "til l1: " << l1 << endl;
				double kareas = timer.getElapsedTimeInMilliSec();
				double keyframeArea = keyframePoly.area_of_polygon(keyframePoly.pointList);
				double keyframeArea1 = keyframePoly.area_of_polygon1(keyframePoly.pointList);
				double kareae = timer.getElapsedTimeInMilliSec();
				double karead = kareae - kareas;
				double l2 = kareae - ls;
				//conprint << "kf area time: " << karead << endl;
				//conprint << "til l2: " << l2 << endl;
				//conprint << "kf area: " << keyframeArea << endl;
				double iareas = timer.getElapsedTimeInMilliSec();
				double interArea = interPoly.area_of_polygon(interPoly.pointList);
				double interArea1 = interPoly.area_of_polygon1(interPoly.pointList);
				double iareae = timer.getElapsedTimeInMilliSec();
				double iaread = iareae - iareas;
				double l3 = iareae - ls;
				//conprint << "inter area time: " << iaread << endl;
				//conprint << "til l3: " << l3 << endl;
				//conprint << "inter area: " << interArea << endl;
				double alpha = interArea / screenArea + interArea / keyframeArea;
				//double alpha1 = interArea1 / screenArea + interArea1 / keyframeArea1;
				AlphaState as;
				as.alpha = alpha;
				as.index = i;
				alphaList.push_back(as);
				//conprint << "keyframe " << i << endl;
				//conprint << keyframeArea << endl;
				//conprint << "inter area" << endl;
				//conprint << interArea << endl;
				//conprint << "alpha" << endl;
				//conprint << alpha << endl;
				//double le = timer.getElapsedTimeInMilliSec();
				//double ld = le - ls;
				//conprint << "loop: " << i << "  " << ld << endl;
			}
			double polyEnd = timer.getElapsedTimeInMilliSec();
			double polyDuration = polyEnd - polyStart;
			conprint << "calc poly time: " << polyDuration << endl;

			if (success)
			{
				//double estiSortStart = timer.getElapsedTimeInMilliSec();
				//int estiIndex = -1;
				//double max = 0;
				//for (int i = 0; i < alphaList.size(); i++)
				//{
				//	if (alphaList[i].alpha>max)
				//	{
				//		max = alphaList[i].alpha;
				//		estiIndex = i;
				//	}
				//}
				//conprint << "before sort esti" << endl;
				//for (int i = 0; i < alphaList.size(); i++)
				//{
				//	conprint << alphaList[i].alpha << "  " << alphaList[i].index << endl;
				//}
				double sorts = timer.getElapsedTimeInMilliSec();
				sort(alphaList.begin(), alphaList.end(), compareAlpha);
				double sorte = timer.getElapsedTimeInMilliSec();
				double sortd = sorte - sorts;
				conprint << "sort: " << sortd << endl;
				//conprint << "after sort esti" << endl;
				//for (int i = 0; i < alphaList.size(); i++)
				//{
				//	conprint << alphaList[i].alpha << "  " << alphaList[i].index << endl;
				//}
				double ps = timer.getElapsedTimeInMilliSec();
				int resultCount = (min)((int)alphaList.size(), indexCount);
				for (int i = 0; i < resultCount; i++)
				{
					indexes.push_back(alphaList[i].index);
				}
				double pe = timer.getElapsedTimeInMilliSec();
				double pd = pe - ps;
				conprint << "push: " << pd << endl;
				//stringstream ss;
				//for (int i = 0; i < indexes.size(); i++)
				//{
				//	ss << indexes[i] << "  ";
				//}
				//str = ss.str();
				//double estiSortEnd = timer.getElapsedTimeInMilliSec();
				//double estiSortDuration = estiSortEnd - estiSortStart;
				//conprint << "esti sort duration" << estiSortDuration << endl;

				//copy vector
				//indexes = estiIdxes;
				return success;// estiIndex;
			}
		}
	}
	else //single scale
	{
		indexes.push_back(0);
		return true;
	}
}

int PatternDetector::matchKeyFrames(Mat& homography, vector<int>& indexes, vector<int>& matchIdxes, vector<int>& estiIdxes, string& str)
{
	indexes.clear();
	matchIdxes.clear();
	estiIdxes.clear();
	if (estimatedHomographyFound)
	{
		conprint << "begin calc areas" << endl;
		MyPoint lt(0, 0), lb(0, screenHeight), rb(screenWidth, screenHeight), rt(screenWidth, 0);
		vector<MyPoint> screenPointList;
		screenPointList.push_back(lt);
		screenPointList.push_back(lb);
		screenPointList.push_back(rb);
		screenPointList.push_back(rt);
		MyPolygon screenPoly(screenPointList);
		double screenArea = screenPoly.area_of_polygon(screenPoly.pointList);
		conprint << "screea area" << endl;
		conprint << screenArea << endl;
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
			conprint << "keyframe " << i << endl;
			conprint << keyframeArea << endl;
			conprint << "inter area" << endl;
			conprint << interArea << endl;
			conprint << "alpha" << endl;
			conprint << alpha << endl;
		}
		double polyEnd = timer.getElapsedTimeInMilliSec();
		double polyDuration = polyEnd - polyStart;
		conprint << "calc poly time: " << polyDuration << endl;

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
			conprint << "before sort esti" << endl;
			for (int i = 0; i < alphaList.size(); i++)
			{
				conprint << alphaList[i].alpha << "  " << alphaList[i].index << endl;
			}
			sort(alphaList.begin(), alphaList.end(), compareAlpha);
			conprint << "after sort esti" << endl;
			for (int i = 0; i < alphaList.size(); i++)
			{
				conprint << alphaList[i].alpha << "  " << alphaList[i].index << endl;
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
			conprint << "esti sort duration" << estiSortDuration << endl;

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
		conprint << "begin match" << endl;
		for (int i = 0; i < size; i++){
			conprint << "begin match keyframe: " << i << endl;
			MatchState ms = matchKeyFrame(i, matchesList[i]);
			conprint << ms.found << ms.num << ms.index;
			matchCount.push_back(ms);
		}
		double matchEnd = timer.getElapsedTimeInMilliSec();
		double matchDuration = matchEnd - matchStart;
		conprint << "calc match time: " << matchDuration << endl;

		stringstream ss;
		ss << "match condition: ";
		conprint << "match count; " << matchCount.size() << endl;
		for (int i = 0; i < matchCount.size(); i++)
		{
			ss << matchCount[i].found << "  " << matchCount[i].num << "  ";
		}
		str = ss.str();
		conprint << "ss: " << str << endl;

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
		conprint << "before sort match" << matchCount.size() << endl;
		for (int i = 0; i < matchCount.size(); i++)
		{
			conprint << matchCount[i].found << "  " << matchCount[i].num << "  " << matchCount[i].index << endl;
		}
		sort(matchCount.begin(), matchCount.end(), compareCount);
		conprint << "after sort match" << matchCount.size() << endl;
		for (int i = 0; i < matchCount.size(); i++)
		{
			conprint << matchCount[i].found << "  " << matchCount[i].num << "  " << matchCount[i].index << endl;
		}
		int resultCount = (min)((int)matchCount.size(), indexCount);
		conprint << "result count" << resultCount << endl;

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
		//conprint << "indexes size: " <<indexes.size()<< endl;
		//for (int i = 0; i < indexes.size(); i++)
		//{
		//	conprint << indexes[i] << "  ";
		//}
		//conprint << endl;

		//conprint << "matchIdxes size: " <<matchIdxes.size()<< endl;
		//for (int i = 0; i < matchIdxes.size(); i++)
		//{
		//	conprint << matchIdxes[i] << "  ";
		//}
		//conprint << endl;

		//conprint << "estiIdxes size: " << estiIdxes.size() << endl;
		//for (int i = 0; i < estiIdxes.size(); i++)
		//{
		//	conprint << estiIdxes[i] << "  ";
		//}
		//conprint << endl;

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

		Timer timer;
		timer.start();
		conprint << "begin calc areas" << endl;
		MyPoint lt(0, 0), lb(0, screenHeight), rb(screenWidth, screenHeight), rt(screenWidth, 0);
		vector<MyPoint> screenPointList;
		screenPointList.push_back(lt);
		screenPointList.push_back(lb);
		screenPointList.push_back(rb);
		screenPointList.push_back(rt);
		MyPolygon screenPoly(screenPointList);

		double areas = timer.getElapsedTimeInMilliSec();
		double screenArea = screenHeight*screenWidth;// screenPoly.area_of_polygon(screenPoly.pointList);
		double areae = timer.getElapsedTimeInMilliSec();
		double aread = areae - areas;
		conprint << "screen area time: " << aread << endl;
		conprint << "screea area" << endl;
		conprint << screenArea << endl;
		vector<AlphaState> alphaList;
		bool success = true;
		
		//test
		vector<MyPoint> patternPointList;
		PatternTrackingInfo info;
		info.homography = homography;
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
		for (int j = 0; j < 4; j++)
		{
			MyPoint p(info.points2d[j].x, info.points2d[j].y);
			patternPointList.push_back(p);
		}
		MyPolygon patternPoly(patternPointList);
		if(!patternPoly.issimple() || !patternPoly.isconvex())
		{
			success = false;
			return 0;
		}

		double polyStart = timer.getElapsedTimeInMilliSec();
		conprint << "begin calc all" << endl;
		for (int i = 0; i < m_pattern.keyframeList.size(); i++)
		{
			double ls = timer.getElapsedTimeInMilliSec();
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
			/*if (!keyframePoly.issimple() || !keyframePoly.isconvex())
			{
				success = false;
				break;
			}*/
			MyPolygon interPoly;
			double intes = timer.getElapsedTimeInMilliSec();
			bool success = screenPoly.intersect(screenPoly, keyframePoly, interPoly);
			double intee = timer.getElapsedTimeInMilliSec();
			double inted = intee - intes;
			double l1 = intee - ls;
			//conprint << "inter time: " << inted << endl;
			//conprint << "til l1: " << l1 << endl;
			double kareas = timer.getElapsedTimeInMilliSec();
			double keyframeArea = keyframePoly.area_of_polygon(keyframePoly.pointList);
			double keyframeArea1 = keyframePoly.area_of_polygon1(keyframePoly.pointList);
			double kareae = timer.getElapsedTimeInMilliSec();
			double karead = kareae - kareas;
			double l2 = kareae - ls;
			//conprint << "kf area time: " << karead << endl;
			//conprint << "til l2: " << l2 << endl;
			//conprint << "kf area: " << keyframeArea<< endl;
			double iareas = timer.getElapsedTimeInMilliSec();
			double interArea = interPoly.area_of_polygon(interPoly.pointList);
			double interArea1 = interPoly.area_of_polygon1(interPoly.pointList);
			double iareae = timer.getElapsedTimeInMilliSec();
			double iaread = iareae - iareas;
			double l3 = iareae - ls;
			//conprint << "inter area time: " << iaread << endl;
			//conprint << "til l3: " << l3 << endl;
			//conprint << "inter area: " << interArea<< endl;
			double alpha = interArea / screenArea + interArea / keyframeArea;
			//double alpha1 = interArea1 / screenArea + interArea1 / keyframeArea1;
			AlphaState as;
			as.alpha = alpha;
			as.index = i;
			alphaList.push_back(as);
			//conprint << "keyframe " << i << endl;
			//conprint << keyframeArea << endl;
			//conprint << "inter area" << endl;
			//conprint << interArea << endl;
			//conprint << "alpha" << endl;
			//conprint << alpha << endl;
			//double le = timer.getElapsedTimeInMilliSec();
			//double ld = le - ls;
			//conprint << "loop: " << i << "  " << ld << endl;
		}
		double polyEnd = timer.getElapsedTimeInMilliSec();
		double polyDuration = polyEnd - polyStart;
		conprint << "calc poly time: " << polyDuration << endl;

		if (success)
		{
			//double estiSortStart = timer.getElapsedTimeInMilliSec();
			//int estiIndex = -1;
			//double max = 0;
			//for (int i = 0; i < alphaList.size(); i++)
			//{
			//	if (alphaList[i].alpha>max)
			//	{
			//		max = alphaList[i].alpha;
			//		estiIndex = i;
			//	}
			//}
			//conprint << "before sort esti" << endl;
			//for (int i = 0; i < alphaList.size(); i++)
			//{
			//	conprint << alphaList[i].alpha << "  " << alphaList[i].index << endl;
			//}
			double sorts = timer.getElapsedTimeInMilliSec();
			sort(alphaList.begin(), alphaList.end(), compareAlpha);
			double sorte = timer.getElapsedTimeInMilliSec();
			double sortd = sorte - sorts;
			conprint << "sort: " << sortd << endl;
			//conprint << "after sort esti" << endl;
			//for (int i = 0; i < alphaList.size(); i++)
			//{
			//	conprint << alphaList[i].alpha << "  " << alphaList[i].index << endl;
			//}
			double ps = timer.getElapsedTimeInMilliSec();
			int resultCount = (min)((int)alphaList.size(), indexCount);
			cout<<resultCount<<"  "<<alphaList.size()<<"  "<<indexCount<<endl;
			for (int i = 0; i < resultCount; i++)
			{
				indexes.push_back(alphaList[i].index);
				estiIdxes.push_back(alphaList[i].index);
			}
			double pe = timer.getElapsedTimeInMilliSec();
			double pd = pe - ps;
			conprint << "push: " << pd << endl;
			cout<<"fuck: "<<resultCount<<"  "<<indexes.size()<<endl;
			//stringstream ss;
			//for (int i = 0; i < indexes.size(); i++)
			//{
			//	ss << indexes[i] << "  ";
			//}
			//str = ss.str();
			//double estiSortEnd = timer.getElapsedTimeInMilliSec();
			//double estiSortDuration = estiSortEnd - estiSortStart;
			//conprint << "esti sort duration" << estiSortDuration << endl;

			//copy vector
			//indexes = estiIdxes;
			return 0;// estiIndex;
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
	conprint << "after get match : " << matches.size() << endl;
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;

	// Find homography transformation and detect good matches
	double homographyStart = timer.getElapsedTimeInMilliSec();
	bool homographyFound = refineMatchesWithHomography(
		m_queryKeypoints,
		keyframe.keypoints,
		homographyReprojectionThreshold,
		matches,
		m_lastHomography);
	conprint << "after homography: " << matches.size() << endl;
	double homographyEnd = timer.getElapsedTimeInMilliSec();
	double homographyDuration = homographyEnd - homographyStart;
	conprint << "homography: " << homographyDuration << endl;
	MatchState ms;
	ms.found = homographyFound;
	ms.num = matches.size();
	ms.index = index;
	return ms;
}

bool PatternDetector::OpticalTracking(Mat& image, PatternTrackingInfo& info)
{
	branch = Opt;
	conprint << "begin optical tracking" << endl;
	bool homographyFound = false;
	// 2. track features
	conprint << "before optical flow: " << before.size() << endl;
	conprint << "before optical flow: " << before.size() << endl;
	InputArray _prevPts = (InputArray)before;
	Mat prevPtsMat = _prevPts.getMat();

	int npoints = prevPtsMat.checkVector(2, CV_32F, true);
	//conprint<<npoints<<endl;
	conprint << npoints << endl;
	Timer timer;
	KLTTimer kltTimer;
	timer.start();
	double kltStart = timer.getElapsedTimeInMilliSec();
	calcOpticalFlowPyrLK(m_grayImgPrev, m_grayImg, // 2 consecutive images
		before, // input point position in first image
		after, // output point postion in the second image
		status,    // tracking success
		err);      // tracking error
	double kltEnd = timer.getElapsedTimeInMilliSec();
	double kltDuration = kltEnd - kltStart;
	kltTimer.klt = kltDuration;

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
	conprint << "k: " << k << endl;
	Mat homography;
	if (k >= 4)
	{
		// Find homography matrix and get inliers mask
		vector<unsigned char> inliersMask(before.size());
		double findHomographyStart = timer.getElapsedTimeInMilliSec();
		homography = findHomography(patternPoints,
			after,
			0,
			homographyReprojectionThreshold,
			inliersMask);

		double findHomographyEnd = timer.getElapsedTimeInMilliSec();
		double findHomographyDuration = findHomographyEnd - findHomographyStart;
		kltTimer.ransac = findHomographyDuration;
		conprint << "find homography duration: " << findHomographyDuration << endl;

		homographyFound = (countNonZero(inliersMask) > 8);
	}
	else
	{
		homographyFound = false;
	}

	float err = arerror.computeError(*this, homography, branch);
	conprint << "err: " << err << endl;
	if (err > 3.0)
	{
		homographyFound = false;
	}



	
	if (homographyFound){
		info.homography = homography;


		//printMat("info.homography",info.homography);
		//printMat("rough homography", m_lastHomography);
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
		vector<int> indexes;
		bool matchSuccess = matchKeyframesWithPolygon(info.homography, indexes);
		if (matchSuccess)
		{
			nowMatchedKeyframes = indexes;
		}
		drawing.drawContours(*this, image, info, nowMatchedKeyframes);
		m_lostFrameNum = 0;
		estimatedHomographyFound = true;
		//float err = arerror.computeError(*this, homography,branch);
		//conprint << "err: " << err << endl;
		// 3. handle the accepted tracked points
		Scalar scalar(255, 0, 0);
		handleTrackedPoints(image, image, scalar);

		//set estimation
		m_estimatedHomography = info.homography;
		m_initialHomography = info.homography;

		// 4. current points and image become previous ones
		double storeStart = timer.getElapsedTimeInMilliSec();
		swap(before, after);
		swap(m_grayImgPrev, m_grayImg);
		double storeEnd = timer.getElapsedTimeInMilliSec();
		double storeDuration = storeEnd - storeStart;
		kltTimer.store = storeDuration;
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);

		m_lastHomography = info.homography;


		//store timer
		kltTimerList.push_back(kltTimer);
	}
	else
	{
		info.homography = m_lastHomography;
		m_lostFrameNum++;
		if (m_lostFrameNum >= maxLostFrames){
			estimatedHomographyFound = false;
		}
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
	conprint << "begin warped tracking" << endl;
	bool homographyFound = false;
	Branch branch = Wapnew;
	//LOGE("homography found");
	//LOGE("type:%d, rows:%d, cold:%d",m_lastHomography.type(),m_lastHomography.rows,m_lastHomography.cols);
	// Warp image using found homography
	Timer timer;
	double warpStart = timer.getElapsedTimeInMilliSec();
	warpPerspective(m_grayImg, m_warpedImg, m_lastHomography, m_pattern.size, WARP_INVERSE_MAP);
	//warpPerspective(m_grayImg, m_warpedImg, m_lastHomography, m_pattern.size, WARP_INVERSE_MAP | INTER_CUBIC);
	double warpEnd = timer.getElapsedTimeInMilliSec();
	double warpDuration = warpEnd - warpStart;
	conprint << "warp image: " << warpDuration << endl;

	// Get refined matches:
	vector<KeyPoint> warpedKeypoints;
	vector<DMatch> warpedMatches;

	// Detect features on warped image
	double extractStart = timer.getElapsedTimeInMilliSec();
	extractFeatures(m_warpedImg, warpedKeypoints, m_queryDescriptors);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	double extractDuration = extractEnd - extractStart;
	conprint << "extract feature: " << extractDuration << endl;

	// Match with pattern
	double matchStart = timer.getElapsedTimeInMilliSec();
	conprint << "begin get matches" << endl;

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
	conprint << "end get matches" << endl;
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;
	conprint << m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size() << endl;
	conprint << m_pattern.keypoints.size() << endl;

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
	conprint << "homography : " << homographyDuration << endl;

	/*if(homographyFound){
	LOGE("refine homography found");
	}
	else{
	LOGE("refine homography not found");
	}*/

	// Get a result homography as result of matrix product of refined and rough homographies:
	if (homographyFound)
	{
		Mat mulHomography = m_lastHomography*m_refinedHomography;//don't know why but can't use info.homography = m_lastHomography * m_refinedHomography
		info.homography = mulHomography;


		//printMat("info.homography",info.homography);
		//printMat("rough homography", m_lastHomography);
		//printMat("refine homography",m_refinedHomography);
		//printMat("mul homography", mul);


		//LOGE("draw rough homography");
		perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);
		//info.draw2dContour(image, CV_RGB(0,200,0));

		// Transform contour with precise homography
		//LOGE("draw refine homography");
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
		//#if _DEBUG
		info.draw2dContour(image, CV_RGB(200, 0, 200));
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
		m_lostFrameNum = 0;
		estimatedHomographyFound = true;

		float err = arerror.computeError(*this, m_lastHomography, branch);
		conprint << "end compute error" << endl;
		m_lastHomography = info.homography;
	}
	else
	{
		info.homography = m_lastHomography;
		m_lostFrameNum++;
		if (m_lostFrameNum >= maxLostFrames){
			estimatedHomographyFound = false;
		}
	}
	conprint << "warped matches size: " << warpedMatches.size() << endl;
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
	conprint << "begin handle tracked points outer" << endl;
	handleTrackedPoints(image, image);
	m_estimatedHomography = info.homography;
	m_initialHomography = info.homography;
	return homographyFound;
}

bool PatternDetector::warpedTrackingSimple(Mat& image, PatternTrackingInfo& info)
{
	branch = Wapsim;
	bool homographyFound = false;
	//LOGE("homography found");
	//LOGE("type:%d, rows:%d, cold:%d",m_lastHomography.type(),m_lastHomography.rows,m_lastHomography.cols);
	// Warp image using found homography
	Timer timer;
	double warpSimpleStart = timer.getElapsedTimeInMilliSec();
	double warpStart = timer.getElapsedTimeInMilliSec();
	warpPerspective(m_grayImg, m_warpedImg, m_lastHomography, m_pattern.size, WARP_INVERSE_MAP);
	processWarpSignal(m_grayImg, m_warpedImg);
	//warpPerspective(m_grayImg, m_warpedImg, m_lastHomography, m_pattern.size, WARP_INVERSE_MAP | INTER_CUBIC);
	double warpEnd = timer.getElapsedTimeInMilliSec();
	double warpDuration = warpEnd - warpStart;
	conprint << "warp image: " << warpDuration << endl;

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
	conprint << "extract feature: " << extractDuration << endl;

	// Match with pattern
	double matchStart = timer.getElapsedTimeInMilliSec();
	conprint << "begin get matches" << endl;

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
	conprint << "end get matches" << endl;
	//if (indexes.size() != 0)
	//{
	//	nowMatchedKeyframes = indexes;
	//}
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;
	conprint << m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size() << endl;
	conprint << m_pattern.keypoints.size() << endl;

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
	conprint << "homography : " << homographyDuration << endl;


	if (homographyFound)
	{
		Mat mulHomography = m_lastHomography*m_refinedHomography;//don't know why but can't use info.homography = m_lastHomography * m_refinedHomography
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

		perspectiveTransform(warpedPoints, queryPoints, m_lastHomography);
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
		float err = arerror.computeError(*this, m_lastHomography,branch);
		conprint << "end compute error" << endl;


		//printMat("info.homography",info.homography);
		//printMat("rough homography", m_lastHomography);
		//printMat("refine homography",m_refinedHomography);
		//printMat("mul homography", mul);


		//LOGE("draw rough homography");
		perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);
		info.draw2dContour(image, CV_RGB(0, 200, 0));

		// Transform contour with precise homography
		//LOGE("draw refine homography");
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
		//#if _DEBUG
		info.draw2dContour(image, CV_RGB(200, 0, 200));
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);


		conprint << "begin draw contours" << endl;
		
		//match keyframes
		vector<int> indexes;
		vector<int> matchindexes;
		vector<int> estiindexes;
		string str;
		m_lastHomography = info.homography;
		matchKeyframesWithPolygon(m_lastHomography, indexes);
		//matchKeyFrames(m_lastHomography, indexes,matchindexes,estiindexes,str);
		nowMatchedKeyframes = indexes;

		
		drawing.drawContours(*this, image, info, indexes);
		conprint << "end draw contours" << endl;
		//drawing.draw2Contours(*this, image, info, matchIndexes, estiIndexes);
		perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);


		m_lostFrameNum = 0;
		estimatedHomographyFound = true;



		
	}
	else
	{
		info.homography = m_lastHomography;
		m_lostFrameNum++;
		if (m_lostFrameNum >= maxLostFrames){
			estimatedHomographyFound = false;
		}
	}
	conprint << "warped matches size: " << warpedMatches.size() << endl;
	conprint << "begin handle tracked points outer" << endl;
	handleTrackedPoints(image, image);
	m_grayImgPrev = m_grayImg.clone();
	double warpSimpleEnd = timer.getElapsedTimeInMilliSec();
	double warpSimpleDuration = warpSimpleEnd - warpSimpleStart;
	conprint << "warp simple duration: " << warpSimpleDuration << endl;
	return homographyFound;
}


void PatternDetector::processWarpSignal(Mat& ori, Mat& warped)
{
	conprint << "process warp signal: " << isPrintWarp << endl;
	if (isPrintWarp)
	{
		conprint << "begin print warp" << endl;
		string warpedPath = "/sdcard/";
		string oriName = warpedPath + "ori.jpg";
		string warpedName = warpedPath + "warped.jpg";
		imwrite(oriName, ori);
		imwrite(warpedName, warped);
		isPrintWarp = false;
		conprint << "end print warp" << endl;
	}
}

bool PatternDetector::warpedTrackingNew(Mat& image, PatternTrackingInfo& info)
{
	bool homographyFound = false;
	homographyFound = warpedTrackingSimple(image, info);
	if (homographyFound){
		return true;
	}
	//LOGE("homography found");
	//LOGE("type:%d, rows:%d, cold:%d",m_lastHomography.type(),m_lastHomography.rows,m_lastHomography.cols);
	// Warp image using found homography
	if (true)
	{
		branch = Wapnew;
		Timer timer;
		double warpStart = timer.getElapsedTimeInMilliSec();
		warpPerspective(m_grayImg, m_warpedImg, m_lastHomography, m_pattern.size, WARP_INVERSE_MAP);
		processWarpSignal(m_grayImg, m_warpedImg);
		//warpPerspective(m_grayImg, m_warpedImg, m_lastHomography, m_pattern.size, WARP_INVERSE_MAP | INTER_CUBIC);
		double warpEnd = timer.getElapsedTimeInMilliSec();
		double warpDuration = warpEnd - warpStart;
		conprint << "warp image: " << warpDuration << endl;

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
		conprint << "extract feature: " << extractDuration << endl;

		// Match with pattern
		double matchStart = timer.getElapsedTimeInMilliSec();
		conprint << "begin get matches" << endl;

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
		conprint << "end get matches" << endl;
		if (indexes.size() != 0)
		{
			nowMatchedKeyframes = indexes;
		}
		double matchEnd = timer.getElapsedTimeInMilliSec();
		double matchDuration = matchEnd - matchStart;
		conprint << m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size() << endl;
		conprint << m_pattern.keypoints.size() << endl;

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
		conprint << "homography : " << homographyDuration << endl;


		/*if(homographyFound){
		LOGE("refine homography found");
		}
		else{
		LOGE("refine homography not found");
		}*/

		// Get a result homography as result of matrix product of refined and rough homographies:
		if (homographyFound)
		{
			Mat mulHomography = m_lastHomography*m_refinedHomography;//don't know why but can't use info.homography = m_lastHomography * m_refinedHomography
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

			perspectiveTransform(warpedPoints, queryPoints, m_lastHomography);
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
			float err = arerror.computeError(*this, m_lastHomography,branch);
			conprint << "end compute error" << endl;


			//printMat("info.homography",info.homography);
			//printMat("rough homography", m_lastHomography);
			//printMat("refine homography",m_refinedHomography);
			//printMat("mul homography", mul);


			//LOGE("draw rough homography");
			perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);
			info.draw2dContour(image, CV_RGB(0, 200, 0));

			// Transform contour with precise homography
			//LOGE("draw refine homography");
			perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);
			//#if _DEBUG
			info.draw2dContour(image, CV_RGB(200, 0, 200));
			perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);


			conprint << "begin draw contours" << endl;
			m_lastHomography = info.homography;
			drawing.drawContours(*this, image, info, indexes);
			conprint << "end draw contours" << endl;
			//drawing.draw2Contours(*this, image, info, matchIndexes, estiIndexes);
			perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);


			m_lostFrameNum = 0;
			estimatedHomographyFound = true;
		}
		else
		{
			info.homography = m_lastHomography;
			m_lostFrameNum++;
			if (m_lostFrameNum >= maxLostFrames){
				estimatedHomographyFound = false;
			}
		}
		conprint << "warped matches size: " << warpedMatches.size() << endl;
		conprint << "begin handle tracked points outer" << endl;
		handleTrackedPoints(image, image);
		m_grayImgPrev = m_grayImg.clone();
		return homographyFound;
	}
	return homographyFound;
}

//without poly
bool PatternDetector::simpleTracking(Mat& image, PatternTrackingInfo& info)
{
	conprint << "begin simple tracking" << endl;
	Branch branch = Sim;
	bool homographyFound = false;
	TrackerTimer trackerTimer;
	conprint << "estimated homography not found" << endl;
	// Extract feature points from input gray image
	Timer timer;
	timer.start();
	double extractStart = timer.getElapsedTimeInMilliSec();
	conprint << "m_grayImage: " << m_grayImg.cols << "  " << m_grayImg.rows << endl;
	extractFeaturesWithTimer(m_grayImg, m_queryKeypoints, m_queryDescriptors,trackerTimer);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	double extractDuration = extractEnd - extractStart;
	conprint << "extract: " << extractDuration << endl;

	// Get matches with current pattern
	double matchStart = timer.getElapsedTimeInMilliSec();
	double matchptsStart = timer.getElapsedTimeInMilliSec();
	conprint << "begin get matches" << endl;

	vector<int> indexes;
	indexes.clear();
	vector<int> estiIndexes;
	vector<int> matchIndexes;
	string matchstr;
	conprint << "begin match keyframes" << endl;
	m_pattern.keyframeIndex = 0;// = matchKeyFrames(m_estimatedHomography, indexes, matchIndexes, estiIndexes, matchstr);
	for (int i = 0; i < m_pattern.keyframeList.size(); i++)
	{
		indexes.push_back(i);
	}
	//one is here one is num = 1000 important
	getMatches(m_queryDescriptors, indexes, m_matches);
	getMatches(m_queryDescriptors, indexes, m_matches);
	getMatches(m_queryDescriptors, indexes, m_matches);
	conprint << "end match keyframes" << endl;
	conprint << m_pattern.keyframeIndex << endl;
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
	conprint << "get indexes" << endl;
	conprint << "indexes size: " << indexes.size() << endl;
	for (int i = 0; i < indexes.size(); i++)
	{
		conprint << indexes[i] << endl;
	}
	conprint << "match indeses size: " << matchIndexes.size() << endl;
	for (int i = 0; i < matchIndexes.size(); i++)
	{
		conprint << matchIndexes[i] << endl;
	}
	conprint << "esti indexes size: " << estiIndexes.size() << endl;
	for (int i = 0; i < estiIndexes.size(); i++)
	{
		conprint << estiIndexes[i] << endl;
	}


	
	getMatches(m_queryDescriptors, indexes, m_matches);
	double matchptsEnd = timer.getElapsedTimeInMilliSec();
	double matchptsDuration = matchptsEnd - matchptsStart;
	trackerTimer.matchpt = matchptsDuration;
	conprint << indexes.size() << endl;
	if (indexes.size() != 0)
	{
		nowMatchedKeyframes = indexes;
	}
	conprint << m_queryDescriptors.size() << "  " << m_pattern.keyframeList[indexes[0]].descriptors.size() << endl;
	conprint << "before homo size: " << m_matches.size() << endl;
	conprint << "end get matches: " << m_matches.size() << endl;
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;
	//conprint<<m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size()<<endl;
	//conprint<<m_pattern.keypoints.size()<<endl;
	double homographyStart = timer.getElapsedTimeInMilliSec();
	vector<KeyPoint> patternKeyPoints;
	patternKeyPoints.clear();
	for (int i = 0; i < indexes.size(); i++)
	{
		patternKeyPoints.insert(patternKeyPoints.end(), m_pattern.keyframeList[indexes[i]].keypoints.begin(), m_pattern.keyframeList[indexes[i]].keypoints.end());
	}

	// Find homography transformation and detect good matches
	
	//if(isMultiScale){
	homographyFound = refineMatchesWithHomography(
		m_queryKeypoints,
		//m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints,
		patternKeyPoints,
		homographyReprojectionThreshold,
		m_matches,
		m_lastHomography);
	conprint << "homography found is: " << homographyFound << "  " << m_matches.size() << endl;
	conprint << "fuck" << endl;
	//conprint << m_queryKeypoints.size() << "  " << m_pattern.keyframeList[indexes[0]].keypoints.size() << endl;
	conprint << "after homo size: " << m_matches.size() << endl;
	//}
	//else
	//{
	//	homographyFound = refineMatchesWithHomography(
	//			m_queryKeypoints,
	//			m_pattern.keypoints,
	//			homographyReprojectionThreshold,
	//			m_matches,
	//			m_lastHomography);
	//}
	double homographyEnd = timer.getElapsedTimeInMilliSec();
	double homographyDuration = homographyEnd - homographyStart;
	trackerTimer.ransac = homographyDuration;
	conprint << "homography: " << homographyDuration << endl;


	// Transform contour with rough homography
	if (homographyFound){

		info.homography = m_lastHomography;

		m_estimatedHomography = info.homography;
		m_initialHomography = info.homography;
		conprint << "begin compute error" << endl;

		conprint << "matches size: " << m_matches.size() << endl;
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
		float err = arerror.computeError(*this, m_lastHomography,branch);
		conprint << "end compute error" << endl;
		//if(isMultiScale)
		{
			/*perspectiveTransform(m_pattern.keyframeList[m_pattern.keyframeIndex].points2d, info.points2d, m_lastHomography);
			info.draw2dContour(image, CV_RGB(200,0,0));
			conprint<<"keyframe size: "<<m_pattern.keyframeList.size()<<endl;
			for(int i=0;i<m_pattern.keyframeList.size();i++){
			perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, m_lastHomography);
			info.draw2dContour(image, CV_RGB(200,200,200));

			}*/
			//drawContours(image, info);

			conprint << "begin draw contours" << endl;
			drawing.drawContours(*this,image, info, indexes);
			conprint << "end draw contours" << endl;
			m_lastHomography = info.homography;
			//drawing.draw2Contours(*this, image, info, matchIndexes, estiIndexes);
			perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);

		}
		//else
		//{
		//	perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);
		//}
		perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);
		//info.draw2dContour(image, CV_RGB(200,0,0));
		m_lostFrameNum = 0;
		estimatedHomographyFound = true;

		//add timer
		trackTimerList.push_back(trackerTimer);
	}
	else
	{
		m_lostFrameNum++;
		if (m_lostFrameNum >= maxLostFrames){
			estimatedHomographyFound = false;
		}
	}
	
	
	//conprint << "err is: " << err << endl;

	conprint << "begin handle tracked points outer" << endl;
	handleTrackedPoints(image, image);
	m_grayImgPrev = m_grayImg.clone();
	return homographyFound;
}

//with poly
bool PatternDetector::simpleTrackingNew(Mat& image, PatternTrackingInfo& info)
{
	branch = SimNew;
	TrackerTimer trackerTimer;
	conprint << "begin simple tracking" << endl;
	bool homographyFound = false;
	conprint << "estimated homography not found" << endl;
	// Extract feature points from input gray image
	Timer timer;
	timer.start();
	double extractStart = timer.getElapsedTimeInMilliSec();
	conprint << "m_grayImage: " << m_grayImg.cols << "  " << m_grayImg.rows << endl;
	extractFeaturesWithTimer(m_grayImg, m_queryKeypoints, m_queryDescriptors, trackerTimer);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	double extractDuration = extractEnd - extractStart;
	conprint << "extract: " << extractDuration << endl;

	// Get matches with current pattern
	double matchStart = timer.getElapsedTimeInMilliSec();
	conprint << "begin get matches" << endl;

	vector<int> indexes;
	indexes.clear();
	vector<int> estiIndexes;
	vector<int> matchIndexes;
	string matchstr;
	conprint << "begin match keyframes" << endl;
	double matchkfStart = timer.getElapsedTimeInMilliSec();
	//if (nowMatchedKeyframes.size() == 0)
	//{
	//	for (int i = 0; i < m_pattern.keyframeList.size(); i++)
	//	{
	//		nowMatchedKeyframes.push_back(i);
	//	}
	//}
	m_pattern.keyframeIndex = matchKeyFramesNew(m_estimatedHomography, indexes, matchIndexes, estiIndexes, matchstr);
	double matchkfEnd = timer.getElapsedTimeInMilliSec();
	double matchkfDuration = matchkfEnd - matchkfStart;
	trackerTimer.matchkf = matchkfDuration;
	conprint << "end match keyframes: "<<trackerTimer.matchkf << endl;
	conprint << m_pattern.keyframeIndex << endl;
	string str = "index size: " + intToString(indexes.size());
	cout<<"index size"<<endl;
	cout<<indexes.size()<<endl;
	for(int i=0;i<indexes.size();i++)
	{
		cout<<indexes[i]<<" ";
	}
	cout<<endl;
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
	conprint << "get indexes" << endl;
	conprint << "indexes size: " << indexes.size() << endl;
	for (int i = 0; i < indexes.size(); i++)
	{
		conprint << indexes[i] << endl;
	}
	conprint << "match indeses size: " << matchIndexes.size() << endl;
	for (int i = 0; i < matchIndexes.size(); i++)
	{
		conprint << matchIndexes[i] << endl;
	}
	conprint << "esti indexes size: " << estiIndexes.size() << endl;
	for (int i = 0; i < estiIndexes.size(); i++)
	{
		conprint << estiIndexes[i] << endl;
	}
	//getMatches(m_queryDescriptors, m_pattern.keyframeIndex,m_matches);
	//use origi
	//indexes.clear();
	//indexes.push_back(0);
	double matchptsStart = timer.getElapsedTimeInMilliSec();
	getMatches(m_queryDescriptors, indexes, m_matches);
	double matchptsEnd = timer.getElapsedTimeInMilliSec();
	double matchptsDuration = matchptsEnd - matchptsStart;
	trackerTimer.matchpt = matchptsDuration;
	conprint << indexes.size() << endl;
	//if (indexes.size() != 0)
	//{
	//	nowMatchedKeyframes = indexes;
	//}
	//conprint << m_queryDescriptors.size() << "  " << m_pattern.keyframeList[indexes[0]].descriptors.size() << endl;
	conprint << "before homo size: " << m_matches.size() << endl;
	conprint << "end get matches: " << m_matches.size() << endl;
	double matchEnd = timer.getElapsedTimeInMilliSec();
	double matchDuration = matchEnd - matchStart;
	//conprint<<m_pattern.keyframeList[m_pattern.keyframeIndex].keypoints.size()<<endl;
	//conprint<<m_pattern.keypoints.size()<<endl;

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
		m_lastHomography);
	conprint << "homography found is: " << homographyFound << "  " << m_matches.size() << endl;
	//conprint << m_queryKeypoints.size() << "  " << m_pattern.keyframeList[indexes[0]].keypoints.size() << endl;
	conprint << "after homo size: " << m_matches.size() << endl;
	//}
	//else
	//{
	//	homographyFound = refineMatchesWithHomography(
	//			m_queryKeypoints,
	//			m_pattern.keypoints,
	//			homographyReprojectionThreshold,
	//			m_matches,
	//			m_lastHomography);
	//}
	double homographyEnd = timer.getElapsedTimeInMilliSec();
	double homographyDuration = homographyEnd - homographyStart;
	trackerTimer.ransac = homographyDuration;
	conprint << "homography: " << homographyDuration << endl;


	// Transform contour with rough homography
	if (homographyFound){

		info.homography = m_lastHomography;

		m_estimatedHomography = info.homography;
		m_initialHomography = info.homography;
		conprint << "begin compute error" << endl;

		conprint << "matches size: " << m_matches.size() << endl;
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
		float err = arerror.computeError(*this, m_lastHomography,branch);
		conprint << "end compute error" << endl;
		//if(isMultiScale)
		{
			/*perspectiveTransform(m_pattern.keyframeList[m_pattern.keyframeIndex].points2d, info.points2d, m_lastHomography);
			info.draw2dContour(image, CV_RGB(200,0,0));
			conprint<<"keyframe size: "<<m_pattern.keyframeList.size()<<endl;
			for(int i=0;i<m_pattern.keyframeList.size();i++){
			perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, m_lastHomography);
			info.draw2dContour(image, CV_RGB(200,200,200));

			}*/
			//drawContours(image, info);

			conprint << "begin draw contours" << endl;
			drawing.drawContours(*this, image, info, indexes);
			conprint << "end draw contours" << endl;
			//drawing.draw2Contours(*this, image, info, matchIndexes, estiIndexes);
			perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);

		}

		//match keyframes
		vector<int> indexes;
		vector<int> matchindexes;
		vector<int> estiindexes;
		string str;
		//matchKeyframesWithPolygon(m_lastHomography, indexes);
		conprint << "match with polygon: " << indexes.size() << endl;
		//matchKeyFrames(m_lastHomography, indexes,matchindexes,estiindexes,str);
		//nowMatchedKeyframes = indexes;
		//else
		//{
		//	perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);
		//}
		perspectiveTransform(m_pattern.points2d, info.points2d, m_lastHomography);
		//info.draw2dContour(image, CV_RGB(200,0,0));
		m_lostFrameNum = 0;
		estimatedHomographyFound = true;

		//add timer
		trackTimerList.push_back(trackerTimer);
	}
	else
	{
		m_lostFrameNum++;
		if (m_lostFrameNum >= maxLostFrames){
			estimatedHomographyFound = false;
		}
	}


	//conprint << "err is: " << err << endl;

	conprint << "begin handle tracked points outer" << endl;
	handleTrackedPoints(image, image);
	m_grayImgPrev = m_grayImg.clone();
	return homographyFound;
}

bool PatternDetector::isCorrectDistance()
{
	/*if (nowDistance > 0.5f&&nowDistance < 2.5f)
	{
		return true;
	}
	return false;*/
	return correctDistance(nowDistance);
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
	conprint << "get gray: " << getGrayDuration << endl;


	//int maxLostFrames = 5;
	m_lastHomography = info.homography;
	bool homographyFound = false;
	//LOGE("before get branch cols:%d, rows:%d",info.homography.cols,info.homography.rows);
	//homographyFound=false;
	conprint << m_queryKeypoints.size() << endl;
	if (!needNewPoints() && enableOpticalFlow&&isCorrectDistance()){
		Timer timer;
		double optStart = timer.getElapsedTimeInMilliSec();
		homographyFound = OpticalTracking(image, info);
		double optEnd = timer.getElapsedTimeInMilliSec();
		double optDuration = optEnd - optStart;
		if (homographyFound)
		{
			trackWithOpt.push_back(optDuration);
		}
		
	}
	if(!homographyFound)///if not success goto this
	{
		if (estimatedHomographyFound && enableWrap&&isCorrectDistance())
		{
			conprint << "begin warped tracking" << endl;
			homographyFound = warpedTrackingNew(image, info);
			
		}
		else
		{
			if (isPoly)
			{
				Timer timer;
				double trackWithPolyStart=timer.getElapsedTimeInMilliSec();
				homographyFound = simpleTrackingNew(image, info);
				double trackWithPolyEnd = timer.getElapsedTimeInMilliSec();
				double trackWithPolyDuration = trackWithPolyEnd - trackWithPolyStart;
				if (homographyFound)
				{
					trackWithPoly.push_back(trackWithPolyDuration);
				}
			}
			else
			{
				Timer timer;
				double trackWithoutPolyStart = timer.getElapsedTimeInMilliSec();
				homographyFound = simpleTracking(image, info);
				double trackWithoutPolyEnd = timer.getElapsedTimeInMilliSec();
				double trackWithoutPolyDuration = trackWithoutPolyEnd - trackWithoutPolyStart;
				if (homographyFound)
				{
					trackWithoutPoly.push_back(trackWithoutPolyDuration);
				}
			}
			
		}
		m_opticalFrameNum = 0;
	}

	//push error
	arerror.pushError(*this,nowError);

	conprint << "end find pattern homography found: " << homographyFound << endl;
	stringstream disstream;
	if (eyes.size() > 0)
	{

		disstream << engine.eye.distance;
		string disstring = disstream.str();
		drawing.drawText(*this, image, disstring, Point(10, 70), Scalar(200, 0, 0));
		nowDistance = engine.eye.distance;
	}

	stringstream indexstream;
	for (int i = 0; i < nowMatchedKeyframes.size(); i++)
	{
		indexstream << nowMatchedKeyframes[i] << "  ";
	}
	string indexstring = indexstream.str();
	drawing.drawText(*this, image, indexstring, Point(10, 90), Scalar(200, 0, 0));


	//conprint now match keyframes
	conprint << "now match keyframes: " << nowMatchedKeyframes.size() << endl;
	conprint << branch << endl;
	for (int i = 0; i < nowMatchedKeyframes.size(); i++)
	{
		conprint << nowMatchedKeyframes[i] << "  ";
	}
	conprint << endl;


	//process time signal
	processTimeSignal();


	//return value
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
	conprint << "detect time: " << detectDuration << endl;

	conprint << "keypoints size: " << keypoints.size() << endl;
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
	conprint << "retain best: " << retainDuration << endl;


	double extractStart = timer.getElapsedTimeInMilliSec();
	m_extractor->compute(image, keypoints, descriptors);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	conprint << "descriptor size: " << descriptors.size() << endl;
	if (keypoints.empty())
		return false;

	return true;
}

bool PatternDetector::extractFeaturesWithTimer(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors, TrackerTimer& trackTimer) const
{
	assert(!image.empty());
	//assert(image.channels() == 1);

	Timer timer;
	timer.start();
	double detectStart = timer.getElapsedTimeInMilliSec();
	m_detector->detect(image, keypoints);
	double detectEnd = timer.getElapsedTimeInMilliSec();
	double detectDuration = detectEnd - detectStart;
	conprint << "detect time: " << detectDuration << endl;
	trackTimer.detect = detectDuration;

	conprint << "keypoints size: " << keypoints.size() << endl;
	if (keypoints.empty())
		return false;

	double retainStart = timer.getElapsedTimeInMilliSec();
	int num = 200;
	if (isPoly)
	{
		//num = 1000;
	}
	if (keypoints.size() > num)
	{
		KeyPointsFilter filter;
		filter.retainBest(keypoints, num);
		//keyframe.keyPoints.resize(num);
	}
	double retainEnd = timer.getElapsedTimeInMilliSec();
	double retainDuration = retainEnd - retainStart;
	conprint << "retain best: " << retainDuration << endl;


	double extractStart = timer.getElapsedTimeInMilliSec();
	m_extractor->compute(image, keypoints, descriptors);
	double extractEnd = timer.getElapsedTimeInMilliSec();
	double extractDuration = extractEnd - extractStart;
	trackTimer.extract = extractDuration;
	conprint << "descriptor size: " << descriptors.size() << endl;
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
			conprint << "multi match start" << endl;
			m_matcher->match(queryDescriptors, m_pattern.keyframeList[m_pattern.keyframeIndex].descriptors, matches);
			conprint << "match size: " << matches.size() << endl;
			conprint << "multi match end" << endl;
		}
		//else
		//{
		//	conprint<<"non multi match start"<<endl;
		//	// Perform regular match
		//	Timer timer;
		//	timer.start();
		//	double regularMatchStart=timer.getElapsedTimeInMilliSec();
		//	m_matcher->match(queryDescriptors, m_pattern.descriptors, matches);
		//	double regularMatchEnd=timer.getElapsedTimeInMilliSec();
		//	double regularMatchDuration=regularMatchEnd-regularMatchStart;
		//	conprint<<"regular match: "<<regularMatchDuration<<endl;
		//	conprint<<"non multi match send"<<endl;
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
		//	conprint<<"regular match: "<<regularMatchDuration<<endl;
		//}
	}
}

void PatternDetector::getMatches(const Mat& queryDescriptors, vector<int>& indexes, vector<DMatch>& matches)
{
	conprint << "begin matches with indexes" << endl;
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
			//Timer timer;
			//timer.start();
			//double sizes = timer.getElapsedTimeInMilliSec();
			//if (indexes.size() == 0)//match failed,push all keyframes into indexes to implement a full match
			//{
			//	int size = m_pattern.keyframeList.size();
			//	for (int i = 0; i < size; i++)
			//	{
			//		indexes.push_back(i);
			//	}
			//}
			//conprint << "begin calc rows" << endl;
			//int cols = m_pattern.descriptors.cols;
			//int type = m_pattern.descriptors.type();
			//int rows = 0;
			//for (int i = 0; i < indexes.size(); i++)
			//{
			//	rows += m_pattern.keyframeList[i].descriptors.rows;
			//}
			//double sizee = timer.getElapsedTimeInMilliSec();
			//double sized = sizee - sizes;
			//conprint << "compute size: " << sized << endl;
			conprint << "begin combine descriptors" << endl;
			Mat descriptors;
			for (int i = 0; i < indexes.size(); i++)
			{
				//Timer timer;
				//timer.start();
				//double pushs = timer.getElapsedTimeInMilliSec();
				conprint << m_pattern.keyframeList.size() << "  " << indexes[i] << endl;
				Mat kfDescriptors = m_pattern.keyframeList[indexes[i]].descriptors;
				conprint << "keyframe: " << i << endl;
				conprint << kfDescriptors.size() << endl;
				descriptors.push_back(kfDescriptors);
				//double pushe = timer.getElapsedTimeInMilliSec();
				//double pushd = pushe - pushs;
				//conprint << "push: " << i << pushd << endl;
			}
			conprint << "end combine descriptors" << endl;
			conprint << descriptors.size() << endl;
			if (descriptors.rows == 0)
			{
				return;
			}
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
		//	conprint<<"regular match: "<<regularMatchDuration<<endl;
		//}
	}
	conprint << "end matches with indexes" << endl;
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
	conprint << "begin refine matches with homography" << endl;
	conprint << "begin matches size: " << matches.size() << endl;
	const int minNumberMatchesAllowed = 8;

	if (matches.size() < minNumberMatchesAllowed)
	{
		return false;
	}

	// Prepare data for findHomography
	vector<Point2f> srcPoints(matches.size());
	vector<Point2f> dstPoints(matches.size());
	conprint << "begin copy" << endl;
	for (size_t i = 0; i < matches.size(); i++)
	{
		srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
		dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
	}

	// Find homography matrix and get inliers mask
	vector<unsigned char> inliersMask(srcPoints.size());
	Timer timer;
	timer.start();
	conprint << "timer start" << endl;
	double findHomographyStart = timer.getElapsedTimeInMilliSec();
	conprint << "begin homography" << endl;
	homography = findHomography(srcPoints,
		dstPoints,
		CV_FM_RANSAC,
		reprojectionThreshold,
		inliersMask);
	conprint << "end homography" << endl;

	double findHomographyEnd = timer.getElapsedTimeInMilliSec();
	double findHomographyDuration = findHomographyEnd - findHomographyStart;
	conprint << "find homography duration: " << findHomographyDuration << endl;

	vector<DMatch> inliers;
	for (size_t i = 0; i<inliersMask.size(); i++)
	{
		if (inliersMask[i])
			inliers.push_back(matches[i]);
	}

	matches.swap(inliers);
	conprint << "matches size: " << matches.size() << endl;
	conprint << "end refine matches with homography" << endl;
	return matches.size() > minNumberMatchesAllowed;
}

// determine which tracked point should be accepted
bool PatternDetector::acceptTrackedPoint(int i) 
{
	return status[i];
}



// handle the currently tracked points
void PatternDetector::handleTrackedPoints(Mat &frame, Mat &output, Scalar scalar) {
	conprint << "begin handle tracked points inner" << endl;
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


void PatternDetector::processTimeSignal()
{
	if (isPrintTime == false)
	{
		return;
	}

	fstream opttimefile;
	opttimefile.open("sdcard/opt.txt", ios::out);
	for (int i = 0; i < trackWithOpt.size(); i++)
	{
		opttimefile << trackWithOpt[i] << endl;
	}
	opttimefile.flush();
	opttimefile.close();

	fstream twptimefile;
	twptimefile.open("sdcard/twp.txt", ios::out);
	//twptimefile << kltTimerList.size() << endl;
	for (int i = 0; i < trackWithPoly.size(); i++)
	{
		twptimefile << trackWithPoly[i] << endl;
	}
	twptimefile.flush();
	twptimefile.close();


	fstream twoptimefile;
	twoptimefile.open("sdcard/twop.txt", ios::out);
	//twoptimefile<<
	for (int i = 0; i < trackWithoutPoly.size(); i++)
	{
		twoptimefile << trackWithoutPoly[i] << endl;
	}
	twoptimefile.flush();
	twoptimefile.close();


	fstream klttimefile;
	klttimefile.open("sdcard/ktf.txt", ios::out);
	for (int i = 0; i < kltTimerList.size(); i++)
	{
		klttimefile << kltTimerList[i] << endl;
	}
	klttimefile.flush();
	klttimefile.close();


	fstream tracktimefile;
	tracktimefile.open("/sdcard/ttf.txt", ios::out);
	for (int i = 0; i < trackTimerList.size(); i++)
	{
		tracktimefile << trackTimerList[i] << endl;
	}
	tracktimefile.flush();
	tracktimefile.close();


	isPrintTime = false;
}

