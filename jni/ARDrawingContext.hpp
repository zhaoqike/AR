
#ifndef ARDrawingContext_HPP
#define ARDrawingContext_HPP

////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"
#include "CameraCalibration.hpp"

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <opencv2/opencv.hpp>
using namespace cv;

void ARDrawingContextDrawCallback(void* param);

class ARDrawingContext {
public:
	ARDrawingContext();
	ARDrawingContext(string windowName, Size frameSize,
			const CameraCalibration& c);
	void init(string windowName, Size frameSize, const CameraCalibration& c);
	~ARDrawingContext();

	bool isPatternPresent;
	Transformation patternPose;

	//! Set the new frame for the background
	void updateBackground(const Mat& frame);

	void updateWindow();

	//private:
	friend void ARDrawingContextDrawCallback(void* param);
	//! Render entire scene in the OpenGl window
	void draw();

	//! Draws the background with video
	void drawCameraFrame();

	//! Draws the AR
	//void drawAugmentedScene();
	void drawAugmentedScene(float x = 0.0f, float y = 0.0f, float z = 0.0f);

	//! Builds the right projection matrix from the camera calibration for AR
	void buildProjectionMatrix(const CameraCalibration& calibration, int w,
			int h, Matrix44& result);

	//! Draws the coordinate axis
	void drawCoordinateAxis();

	//! Draw the cube model
	void drawCubeModel();

	bool drawMesh();

	int width;
	int height;

	//private:
public:
	bool m_isTextureInitialized;
	unsigned int m_backgroundTextureId;
	CameraCalibration m_calibration;
	Mat m_backgroundImage;
	string m_windowName;
};

#endif
