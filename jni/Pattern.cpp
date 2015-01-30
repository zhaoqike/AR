

////////////////////////////////////////////////////////////////////
// File includes:
#include "Pattern.hpp"
#include "Globals.h"

extern bool isMultiScale;
extern bool isShowRects;

Eye::Eye(float _x, float _y, float _z)
{
	x = _x;
	y = _y;
	z = _z;
	distance = sqrt(x*x + y*y + z*z);
}

Eye::Eye()
{
	x = y = z = 0;
}

void Eye::computeDistance()
{
	distance = sqrt(x*x + y*y + z*z);
}

ostream& operator << (ostream& os, Eye& eye) //定义运算符“<<”重载函数
{
	os << eye.x << "  " << eye.y << "  " << eye.z << "  " << eye.distance;
	return os;
}


void PatternTrackingInfo::computePose(const Pattern& pattern, const CameraCalibration& calibration)
{
	Mat Rvec;
	Mat_<float> Tvec;
	Mat raux,taux;
	//if(isMultiScale){
	//	  solvePnP(pattern.keyframeList[pattern.keyframeIndex].points3d, points2d, calibration.getIntrinsic(), calibration.getDistorsion(),raux,taux);
	//}else{
	solvePnP(pattern.points3d, points2d, calibration.getIntrinsic(), calibration.getDistorsion(),raux,taux);
	//}
	raux.convertTo(Rvec,CV_32F);
	taux.convertTo(Tvec ,CV_32F);

	Mat_<float> rotMat(3,3);
	Rodrigues(Rvec, rotMat);

	// Copy to transformation matrix
	for (int col=0; col<3; col++)
	{
		for (int row=0; row<3; row++)
		{
			pose3d.r().mat[row][col] = rotMat(row,col); // Copy rotation component
		}
		pose3d.t().data[col] = Tvec(col); // Copy translation component
	}
	Eye eye(Tvec(0), Tvec(1), Tvec(2));
	eyes.push_back(eye);
	engine.eye = eye;
	// Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
	pose3d = pose3d.getInverted();
}

void PatternTrackingInfo::draw2dContour(Mat& image, Scalar color, int lineWidth) const
{
	if (isShowRects == false)
	{
		return;
	}
	for (size_t i = 0; i < points2d.size(); i++)
	{
		line(image, points2d[i], points2d[ (i+1) % points2d.size() ], color, lineWidth, CV_AA);
	}
}

