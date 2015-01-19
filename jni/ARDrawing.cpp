#include "ARDrawing.h"
#include "Utils.h"
#include "Globals.h"


ARDrawing::ARDrawing()
{
}


ARDrawing::~ARDrawing()
{
}

void ARDrawing::draw2dContour(PatternDetector& pd, Mat& image, PatternTrackingInfo& info, vector<Point2f> points, Mat homography, Scalar color, int lineWidth)
{
	if (isShowRects == false)
	{
		return;
	}
	perspectiveTransform(points, info.points2d, homography);
	info.draw2dContour(image, color, lineWidth);
}

void ARDrawing::drawContours(PatternDetector& pd, Mat& image, PatternTrackingInfo& info)
{
	for (int i = 0; i<pd.m_pattern.keyframeList.size(); i++){
		//perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, m_roughHomography);
		draw2dContour(pd,image, info, pd.m_pattern.keyframeList[i].points2d, pd.m_roughHomography, CV_RGB(200, 200, 200));
	}
	//perspectiveTransform(m_pattern.keyframeList[m_pattern.keyframeIndex].points2d, info.points2d, m_roughHomography);
	draw2dContour(pd, image, info, pd.m_pattern.keyframeList[pd.m_pattern.keyframeIndex].points2d, pd.m_roughHomography, CV_RGB(200, 200, 200));

	perspectiveTransform(pd.m_pattern.points2d, info.points2d, pd.m_roughHomography);
}

void ARDrawing::drawContours(PatternDetector& pd, Mat& image, PatternTrackingInfo& info, vector<int> indexes)
{
	for (int i = 0; i<pd.m_pattern.keyframeList.size(); i++){
		//perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, m_roughHomography);
		//info.draw2dContour(image, CV_RGB(200, 200, 200));
		draw2dContour(pd, image, info, pd.m_pattern.keyframeList[i].points2d, pd.m_roughHomography, CV_RGB(200, 200, 200));
	}
	for (int i = 0; i < indexes.size(); i++)
	{
		//perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, m_roughHomography);
		//info.draw2dContour(image, CV_RGB(200, 0, 0));
		draw2dContour(pd, image, info, pd.m_pattern.keyframeList[i].points2d, pd.m_roughHomography, CV_RGB(200, 200, 200));
	}


	perspectiveTransform(pd.m_pattern.points2d, info.points2d, pd.m_roughHomography);
}

void ARDrawing::draw2Contours(PatternDetector& pd, Mat& image, PatternTrackingInfo& info, vector<int> matchIndexes, vector<int> estiIndexes)
{
	string str = "size: " + IntToString(matchIndexes.size()) + "  ";
	for (int i = 0; i < matchIndexes.size(); i++)
	{
		str += IntToString(matchIndexes[i]) + "  ";
	}
	putText(image, str, Point(10, 30), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 200, 0));
	for (int i = 0; i<pd.m_pattern.keyframeList.size(); i++){
		//perspectiveTransform(m_pattern.keyframeList[i].points2d, info.points2d, m_roughHomography);
		//info.draw2dContour(image, CV_RGB(200, 200, 200));
		draw2dContour(pd, image, info, pd.m_pattern.keyframeList[i].points2d, pd.m_roughHomography, CV_RGB(200, 200, 200));
	}
	for (int i = 0; i < matchIndexes.size(); i++)
	{
		//perspectiveTransform(m_pattern.keyframeList[matchIndexes[i]].points2d, info.points2d, m_roughHomography);
		//info.draw2dContour(image, CV_RGB(200, 0, 0),5);
		draw2dContour(pd, image, info, pd.m_pattern.keyframeList[matchIndexes[i]].points2d, pd.m_roughHomography, CV_RGB(200, 0, 0));
	}
	for (int i = 0; i < estiIndexes.size(); i++)
	{
		//perspectiveTransform(m_pattern.keyframeList[estiIndexes[i]].points2d, info.points2d, m_roughHomography);
		//info.draw2dContour(image, CV_RGB(0, 200, 0));
		draw2dContour(pd, image, info, pd.m_pattern.keyframeList[estiIndexes[i]].points2d, pd.m_roughHomography, CV_RGB(0, 200, 0));
	}

	perspectiveTransform(pd.m_pattern.points2d, info.points2d, pd.m_roughHomography);
}