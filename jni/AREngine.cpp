#include "AREngine.h"
#include "Timer.h"
#include "Globals.h"
#include "PatternFactory.h"

AREngine::AREngine()
{
}


AREngine::~AREngine()
{
}


void AREngine::init()
{
	cout << "begin train pattern" << endl;


	PatternFactory factory;
	factory.buildPattern(bj1);
	Mat patternImage = cv::imread(imagePath);

	if (patternImage.empty())
	{
		std::cout << "Input image cannot be read" << std::endl;
		return;
	}
	cv::Size frameSize(640, 480);
	cout << "begin pipeline init" << endl;
	pipeline.init(patternImage, calibration);
	cout << "begin drawctx init" << endl;
	drawingCtx.init("Markerless AR", frameSize, calibration);
	cout << "end drawctx init" << endl;

	//kalman
	//cout << "begin init kalman" << endl;
	//initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
	//measurements.setTo(Scalar(0));

	cout << "begin load model" << endl;
	//load model
	//PMesh::EdgeCost g_edgemethod = PMesh::QUADRICTRI;
	//g_pMesh = new Mesh("/sdcard/models/apple.ply");
	//vector<vertex>& vert = g_pMesh->_vlist;

	//if (g_pMesh) g_pMesh->Normalize(0.2f);// center mesh around the origin & shrink to fit
	cout << "after normal" << endl;


	//g_pProgMesh = new PMesh(g_pMesh, g_edgemethod );
	cout << "end train" << endl;
}


bool AREngine::processFrame(Mat& cameraFrame)
{
	// Clone image used for background (we will draw overlay on it)

	Timer timer;
	timer.start();
	double cloneStart = timer.getElapsedTimeInMilliSec();
	//cv::Mat img = cameraFrame.clone();
	double cloneEnd = timer.getElapsedTimeInMilliSec();
	double cloneDuration = cloneEnd - cloneStart;
	cout << "clone image: " << cloneDuration << endl;

	// Draw information:

	//cv::putText(img, "RANSAC threshold: " + ToString(pipeline.m_patternDetector.homographyReprojectionThreshold) + "( Use'-'/'+' to adjust)", cv::Point(10, 30), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0,200,0));

	// Set a new camera frame:
	//LOGE("process channel:%d", img.channels());
	double updateStart = timer.getElapsedTimeInMilliSec();
	//drawingCtx.updateBackground(img);
	drawingCtx.width = cameraFrame.cols;
	drawingCtx.height = cameraFrame.rows;
	double updateEnd = timer.getElapsedTimeInMilliSec();
	double updateDuration = updateEnd - updateStart;
	cout << "update background: " << updateDuration << endl;


	// Find a pattern and update it's detection status:
	double pipeStart = timer.getElapsedTimeInMilliSec();
	drawingCtx.isPatternPresent = pipeline.processFrame(cameraFrame);
	double pipeEnd = timer.getElapsedTimeInMilliSec();
	double pipeDuration = pipeEnd - pipeStart;
	cout << "pipeline process frame: " << pipeDuration << endl;


	// Update a pattern pose:
	Transformation patternPose = pipeline.getPatternLocation();


	drawingCtx.patternPose = patternPose;



	bool shouldQuit = false;


	return shouldQuit;
}
