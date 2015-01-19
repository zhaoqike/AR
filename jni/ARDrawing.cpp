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
		draw2dContour(pd, image, info, pd.m_pattern.keyframeList[indexes[i]].points2d, pd.m_roughHomography, CV_RGB(200, 0, 0));
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


void ARDrawing::drawTrackedPoints(PatternDetector &pd, Mat &output, Scalar scalar) {
	cout << "begin handle tracked points inner" << endl;
	if (isShowPoints == false)
	{
		return;
	}
	// for all tracked points
	for (int i = 0; i < pd.after.size(); i++) {
		//cout<<"i: "<<i<<"  "<<points[0][i].x<<"  "<<points[0][i].y<<endl;

		// draw line and circle
		//line(output, initial[i], points[1][i], Scalar(255,255,255));
		circle(output, pd.after[i], 3, scalar, -1);
	}


}

void ARDrawing::drawTexts(PatternDetector& pd, Mat& img, const vector<string>& texts, Point org, Scalar color)
{
	if (isShowTexts == false)
	{
		return;
	}
	for (int i = 0; i < texts.size(); i++)
	{
		Point nowOrg(org.x, org.y + 20 * i);
		putText(img, texts[i], nowOrg, CV_FONT_HERSHEY_PLAIN, 1, color);
	}
}

void ARDrawing::drawText(PatternDetector& pd, Mat& img, const string& text, Point org, Scalar color)
{
	if (isShowTexts == false)
	{
		return;
	}
	putText(img, text, org, CV_FONT_HERSHEY_PLAIN, 1, color);
}