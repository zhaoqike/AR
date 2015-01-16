
#ifndef Example_MarkerBasedAR_CameraCalibration_hpp
#define Example_MarkerBasedAR_CameraCalibration_hpp

////////////////////////////////////////////////////////////////////
// File includes:
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/**
 * A camera calibration class that stores intrinsic matrix and distortion coefficients.
 */
class CameraCalibration {
public:
	CameraCalibration();
	CameraCalibration(float fx, float fy, float cx, float cy);
	CameraCalibration(float fx, float fy, float cx, float cy,
			float distorsionCoeff[5]);

	void getMatrix34(float cparam[3][4]) const;

	const Matx33f& getIntrinsic() const;
	const Mat_<float>& getDistorsion() const;

	float& fx();
	float& fy();

	float& cx();
	float& cy();

	float fx() const;
	float fy() const;

	float cx() const;
	float cy() const;
private:
	Matx33f m_intrinsic;
	Mat_<float> m_distortion;
};

#endif
