#include <jni.h>
#include <android/log.h>
#include "fstream"


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "ARDrawingContext.hpp"
#include "ARPipeline.hpp"
#include "DebugHelpers.hpp"
#include "Timer.h"
#include <thread>
#include <mutex>
#include <unistd.h>
#include "Utils.h"
#include "Model/pmesh.h"
#include "Model/mesh.h"
#include "MutexImage.h"

#define  LOG_TAG    "libgljni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)




using namespace std;
using namespace cv;


bool isRedirect=false;


Mat gray;


/*Mat cameraToFirstRgb;
mutex cameraToFirstLock;
bool cameraToFirstReady=false;

Mat secondToRenderRgb;
mutex secondToRenderLock;
bool secondToRenderReady=false;

Mat firstToSecondRgb;
mutex firstToSecondLock;
bool firstToSecondReady=false;*/

MutexImage cameraToFirst;
MutexImage firstToSecond;
MutexImage secondToThird;
MutexImage thirdToRender;

extern bool isMultiScale;

bool trackState=false;
float val[12];
Point3d center;

extern ARPipeline pipeline;
extern ARDrawingContext drawingCtx;
extern CameraCalibration calibration;

//ARPipeline pipeline;
//ARDrawingContext drawingCtx;




//CameraCalibration calibration(1024.0f, 576.0f, 512.0f, 512.0f);
//CameraCalibration calibration(526.58037684199849f, 524.65577209994706f, 318.41744018680112f, 202.96659047014398f);





//KalmanFilter KF;         // instantiate Kalman Filter
int nStates = 18;            // the number of states
int nMeasurements = 6;       // the number of measured states
int nInputs = 0;             // the number of control actions
double dt = 0.125;           // time between measurements (1/FPS)


extern Mesh *g_pMesh;
extern PMesh *g_pProgMesh;

extern bool isShowRects;
extern bool isShowPoints;



Mat measurements(nMeasurements, 1, CV_64F);
bool good_measurement = false;


extern "C" {
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_trainPatternNative(JNIEnv*, jobject, jstring path);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_trackPatternNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_trackPatternMultiThreadNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba);

//preprocess
JNIEXPORT jint JNICALL Java_com_example_ar_ARNativeLib_preprocessFrameNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_setFirstFlagNative(JNIEnv*, jobject);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_setSecondFlagNative(JNIEnv*, jobject);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_storeMapNative(JNIEnv*, jobject);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_setStartPrepTrackFlagNative(JNIEnv*, jobject);

//tracking
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_setStartTrackFlagNative(JNIEnv*, jobject);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_loadMapNative(JNIEnv*, jobject);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_trackingFrameNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba);


//getpose
JNIEXPORT jboolean JNICALL Java_com_example_ar_ARNativeLib_getGLPoseNative(JNIEnv* env, jobject, jfloatArray arr);
JNIEXPORT jboolean JNICALL Java_com_example_ar_ARNativeLib_getCenterNative(JNIEnv* env, jobject, jfloatArray arr);




//common
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_sendFrameToNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_cycleProcessNative(JNIEnv*, jobject);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_FindFeatures(JNIEnv*, jobject, jlong addrGray, jlong addrRgba);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_processFrameNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_redirectStdOut(JNIEnv*, jobject);

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_storeError(JNIEnv*, jobject);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_showRects(JNIEnv*, jobject);
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_showPoints(JNIEnv*, jobject);
}
//declare
void redirectStdOut();
bool processFrame(cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx);
bool processFrameFirstStage(cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx);
bool processFrameSecondStage(cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx, Mat& descriptors);
bool processFrameThirdStage(cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx, Mat& descriptors);
void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);


JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_trainPatternNative(JNIEnv* env, jobject, jstring path)
{
	redirectStdOut();
	LOGE("begin train pattern");
	cout<<"begin train pattern"<<endl;
	const char* str;
	str = env->GetStringUTFChars(path, false);
	if(str == NULL) {
		return NULL; /* OutOfMemoryError already thrown */
	}
	std::cout << str << std::endl;

	Mat patternImage = cv::imread(str);
	env->ReleaseStringUTFChars(path, str);
	LOGE("pattern channels: %d", patternImage.channels());
	cvtColor(patternImage,patternImage,CV_BGR2RGB);
	if (patternImage.empty())
	{
		std::cout << "Input image cannot be read" << std::endl;
		return;
	}
	cv::Size frameSize(1024, 576);
	LOGE("begin pipeline init");
	pipeline.init(patternImage, calibration);
	LOGE("begin drawctx init");
	drawingCtx.init("Markerless AR", frameSize, calibration);
	LOGE("end drawctx init");


	//kalman
	//cout << "begin init kalman" << endl;
	//initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
	//measurements.setTo(Scalar(0));





	cout<<"begin load model"<<endl;
	//load model
	PMesh::EdgeCost g_edgemethod = PMesh::QUADRICTRI;
	g_pMesh = new Mesh("/sdcard/cow.ply");
	vector<vertex>& vert = g_pMesh->_vlist;

	if (g_pMesh) g_pMesh->Normalize();// center mesh around the origin & shrink to fit
	cout<<"after normal"<<endl;


	g_pProgMesh = new PMesh(g_pMesh, g_edgemethod );
	cout<<"end train"<<endl;

}
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_trackPatternNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
{
	Mat& currentFrame = *(Mat*)addrRgba;
	Mat descriptors;
	LOGE("begin track pattern");
	Timer timer;
	timer.start();
	double start = timer.getElapsedTimeInMilliSec();
	bool shouldQuit = processFrame(currentFrame, pipeline,drawingCtx);




	//bool shouldQuit = processFrameFirstStage(currentFrame, pipeline, drawingCtx);

	//cvtColor(dst,dst,CV_GRAY2RGB);


	//shouldQuit = processFrameSecondStage(currentFrame, pipeline, drawingCtx, descriptors);

	//cvtColor(dst,dst,CV_GRAY2RGB);

	//shouldQuit = processFrameThirdStage(currentFrame, pipeline, drawingCtx, descriptors);
}

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_trackPatternMultiThreadNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
{

	Mat& mGr  = *(Mat*)addrGray;
	Mat& mRgb = *(Mat*)addrRgba;


	//
	/*while(cameraToFirstReady==true)
	{
		//LOGE("not obtain new frame");
		//cameraToFirstLock.unlock();
		usleep(2*1000);
		continue;
	}
	cameraToFirstLock.lock();
	cameraToFirstRgb=mRgb.clone();
	gray=mGr.clone();
	cameraToFirstReady=true;
	LOGE("track obtain new frame");
	cameraToFirstLock.unlock();*/
	LOGE("camera to first");
	//cameraToFirst.input(mRgb);
	Mat tmp;
	//thirdToRender.output(tmp);
	//
	/*LOGE("process new frame begin");
	while(secondToRenderReady==false)
	{
		usleep(2*1000);
		continue;

		//processNewFrame=false;
	}
	secondToRenderLock.lock();
	LOGE("process new frame success");
	mRgb=secondToRenderRgb.clone();
	secondToRenderReady=false;
	secondToRenderLock.unlock();*/
}


/**********************************************************************************************************/
void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{

	KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter

	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));       // set process noise
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2));   // set measurement noise
	setIdentity(KF.errorCovPost, Scalar::all(1));             // error covariance


	/** DYNAMIC MODEL **/

	//  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

	// position
	KF.transitionMatrix.at<double>(0,3) = dt;
	KF.transitionMatrix.at<double>(1,4) = dt;
	KF.transitionMatrix.at<double>(2,5) = dt;
	KF.transitionMatrix.at<double>(3,6) = dt;
	KF.transitionMatrix.at<double>(4,7) = dt;
	KF.transitionMatrix.at<double>(5,8) = dt;
	KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
	KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
	KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);

	// orientation
	KF.transitionMatrix.at<double>(9,12) = dt;
	KF.transitionMatrix.at<double>(10,13) = dt;
	KF.transitionMatrix.at<double>(11,14) = dt;
	KF.transitionMatrix.at<double>(12,15) = dt;
	KF.transitionMatrix.at<double>(13,16) = dt;
	KF.transitionMatrix.at<double>(14,17) = dt;
	KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
	KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
	KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);


	/** MEASUREMENT MODEL **/

	//  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	//  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	//  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	//  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
	//  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
	//  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

	KF.measurementMatrix.at<double>(0,0) = 1;  // x
	KF.measurementMatrix.at<double>(1,1) = 1;  // y
	KF.measurementMatrix.at<double>(2,2) = 1;  // z
	KF.measurementMatrix.at<double>(3,9) = 1;  // roll
	KF.measurementMatrix.at<double>(4,10) = 1; // pitch
	KF.measurementMatrix.at<double>(5,11) = 1; // yaw

}

/**********************************************************************************************************/
void updateKalmanFilter( KalmanFilter &KF, Mat &measurement,
		Mat &translation_estimated, Mat &rotation_estimated )
{

	// First predict, to update the internal statePre variable
	Mat prediction = KF.predict();

	// The "correct" phase that is going to use the predicted value and our measurement
	Mat estimated = KF.correct(measurement);

	// Estimated translation
	translation_estimated.at<double>(0) = estimated.at<double>(0);
	translation_estimated.at<double>(1) = estimated.at<double>(1);
	translation_estimated.at<double>(2) = estimated.at<double>(2);

	// Estimated euler angles
	Mat eulers_estimated(3, 1, CV_64F);
	eulers_estimated.at<double>(0) = estimated.at<double>(9);
	eulers_estimated.at<double>(1) = estimated.at<double>(10);
	eulers_estimated.at<double>(2) = estimated.at<double>(11);

	// Convert estimated quaternion to rotation matrix
	rotation_estimated = euler2rot(eulers_estimated);

}

/**********************************************************************************************************/
void fillMeasurements( Mat &measurements,
		const Mat &translation_measured, const Mat &rotation_measured)
{
	// Convert rotation matrix to euler angles
	Mat measured_eulers(3, 1, CV_64F);
	measured_eulers = rot2euler(rotation_measured);

	// Set measurement to predict
	measurements.at<double>(0) = translation_measured.at<double>(0); // x
	measurements.at<double>(1) = translation_measured.at<double>(1); // y
	measurements.at<double>(2) = translation_measured.at<double>(2); // z
	measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
	measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
	measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}


bool processFrame(cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx)
{
	// Clone image used for background (we will draw overlay on it)
	LOGE("ar process frame");
	Timer timer;
	timer.start();
	double cloneStart=timer.getElapsedTimeInMilliSec();
	//cv::Mat img = cameraFrame.clone();
	double cloneEnd=timer.getElapsedTimeInMilliSec();
	double cloneDuration=cloneEnd-cloneStart;
	cout<<"clone image: "<<cloneDuration<<endl;

	// Draw information:

	//cv::putText(img, "RANSAC threshold: " + ToString(pipeline.m_patternDetector.homographyReprojectionThreshold) + "( Use'-'/'+' to adjust)", cv::Point(10, 30), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0,200,0));

	// Set a new camera frame:
	//LOGE("process channel:%d", img.channels());
	double updateStart=timer.getElapsedTimeInMilliSec();
	//drawingCtx.updateBackground(img);
	drawingCtx.width=cameraFrame.cols;
	drawingCtx.height=cameraFrame.rows;
	double updateEnd=timer.getElapsedTimeInMilliSec();
	double updateDuration=updateEnd-updateStart;
	cout<<"update background: "<<updateDuration<<endl;

	LOGE("begin pipeline process frame");
	// Find a pattern and update it's detection status:
	double pipeStart=timer.getElapsedTimeInMilliSec();
	drawingCtx.isPatternPresent = pipeline.processFrame(cameraFrame);
	double pipeEnd=timer.getElapsedTimeInMilliSec();
	double pipeDuration=pipeEnd-pipeStart;
	cout<<"pipeline process frame: "<<pipeDuration<<endl;

	LOGE("begin pipeline get pattern location");
	// Update a pattern pose:
	Transformation patternPose=pipeline.getPatternLocation();
	/*cout<<"before"<<endl;
	Matrix44 m44=patternPose.getMat44();
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			cout<<m44.mat[i][j]<<"  ";
		}
		cout<<endl;
	}*/


	//kalman filter
	// Get the measured translation
	/*Mat translation_measured(3, 1, CV_64F);
	Vector3& t=patternPose.t();
	for(int i=0;i<3;i++){
		translation_measured.at<double>(i,0)=t.data[i];
	}

	// Get the measured rotation
	Mat rotation_measured(3, 3, CV_64F);
	Matrix33& r=patternPose.r();
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			rotation_measured.at<double>(i,j)=r.mat[i][j];
		}
	}

	// fill the measurements vector
	fillMeasurements(measurements, translation_measured, rotation_measured);


	// Instantiate estimated translation and rotation
	Mat translation_estimated(3, 1, CV_64F);
	Mat rotation_estimated(3, 3, CV_64F);

	// update the Kalman filter with good measurements
	updateKalmanFilter( KF, measurements, translation_estimated, rotation_estimated);

	cout << "end kalman filter" << endl;

	for(int i=0;i<3;i++){
		t.data[i]=translation_estimated.at<double>(i,0);
	}

	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			r.mat[i][j]=rotation_estimated.at<double>(i,j);
		}
	}*/

	drawingCtx.patternPose = patternPose;


	LOGE("end pipeline get pattern location");
	// Request redraw of the window:
	//drawingCtx.updateWindow();

	// Read the keyboard input:
	//int keyCode = cv::waitKey(5);

	bool shouldQuit = false;
	/*if (keyCode == '+' || keyCode == '=')
    {
        pipeline.m_patternDetector.homographyReprojectionThreshold += 0.2f;
        pipeline.m_patternDetector.homographyReprojectionThreshold = min(10.0f, pipeline.m_patternDetector.homographyReprojectionThreshold);
    }
    else if (keyCode == '-')
    {
        pipeline.m_patternDetector.homographyReprojectionThreshold -= 0.2f;
        pipeline.m_patternDetector.homographyReprojectionThreshold = max(0.0f, pipeline.m_patternDetector.homographyReprojectionThreshold);
    }
    else if (keyCode == 'h')
    {
        pipeline.m_patternDetector.enableHomographyRefinement = !pipeline.m_patternDetector.enableHomographyRefinement;
		cout<<"change refinement"<<endl;
    }
    else if (keyCode == 27 || keyCode == 'q')
    {
        shouldQuit = true;
    }*/

	return shouldQuit;
}

bool processFrameFirstStage(cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx)
{
	// Clone image used for background (we will draw overlay on it)
	//LOGE("ar process frame");
	Timer timer;
	timer.start();

	drawingCtx.width=cameraFrame.cols;
	drawingCtx.height=cameraFrame.rows;


	LOGE("begin pipeline process frame first");
	// Find a pattern and update it's detection status:
	double pipeStart=timer.getElapsedTimeInMilliSec();
	//drawingCtx.isPatternPresent = pipeline.processFrameFirstStage(cameraFrame);
	double pipeEnd=timer.getElapsedTimeInMilliSec();
	double pipeDuration=pipeEnd-pipeStart;
	cout<<"pipeline process frame: "<<pipeDuration<<endl;


}

bool processFrameSecondStage(cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx, Mat& descriptors)
{
	//Mat descriptors;
	Timer timer;
	timer.start();
	LOGE("begin pipeline process frame second");
	// Find a pattern and update it's detection status:
	double pipeStart=timer.getElapsedTimeInMilliSec();
	//drawingCtx.isPatternPresent = pipeline.processFrameSecondStage(cameraFrame, descriptors);
	double pipeEnd=timer.getElapsedTimeInMilliSec();
	double pipeDuration=pipeEnd-pipeStart;
	cout<<"pipeline process frame: "<<pipeDuration<<endl;




	bool shouldQuit = false;

	return shouldQuit;
}

bool processFrameThirdStage(cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx, Mat& descriptors)
{
	//Mat descriptors;
	Timer timer;
	timer.start();
	LOGE("begin pipeline process frame third");
	// Find a pattern and update it's detection status:
	double pipeStart=timer.getElapsedTimeInMilliSec();
	//drawingCtx.isPatternPresent = pipeline.processFrameThirdStage(cameraFrame, descriptors);
	double pipeEnd=timer.getElapsedTimeInMilliSec();
	double pipeDuration=pipeEnd-pipeStart;
	cout<<"pipeline process frame: "<<pipeDuration<<endl;

	LOGE("begin pipeline get pattern location third");
	// Update a pattern pose:
	Transformation patternPose=pipeline.getPatternLocation();



	//kalman filter
	// Get the measured translation
	/*Mat translation_measured(3, 1, CV_64F);
	Vector3& t=patternPose.t();
	for(int i=0;i<3;i++){
		translation_measured.at<double>(i,0)=t.data[i];
	}

	// Get the measured rotation
	Mat rotation_measured(3, 3, CV_64F);
	Matrix33& r=patternPose.r();
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			rotation_measured.at<double>(i,j)=r.mat[i][j];
		}
	}

	// fill the measurements vector
	fillMeasurements(measurements, translation_measured, rotation_measured);


	// Instantiate estimated translation and rotation
	Mat translation_estimated(3, 1, CV_64F);
	Mat rotation_estimated(3, 3, CV_64F);

	// update the Kalman filter with good measurements
	updateKalmanFilter( KF, measurements,
			translation_estimated, rotation_estimated);

	cout << "end kalman filter" << endl;

	for(int i=0;i<3;i++){
		t.data[i]=translation_estimated.at<double>(i,0);
	}

	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			r.mat[i][j]=rotation_estimated.at<double>(i,j);
		}
	}*/

	drawingCtx.patternPose = patternPose;


	LOGE("end pipeline get pattern location third");


	bool shouldQuit = false;

	return shouldQuit;
}


//preprocess
JNIEXPORT jint JNICALL Java_com_example_ar_ARNativeLib_preprocessFrameNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
{
	Mat& mGr  = *(Mat*)addrGray;
	Mat& mRgb = *(Mat*)addrRgba;

}

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_setFirstFlagNative(JNIEnv*, jobject){
	//map1.changeColor();
	//preprocessor.Run();
	redirectStdOut();


}

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_setSecondFlagNative(JNIEnv*, jobject)
{

}
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_storeMapNative(JNIEnv*, jobject)
{

}

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_setStartPrepTrackFlagNative(JNIEnv*, jobject)
{

}

//tracker
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_setStartTrackFlagNative(JNIEnv*, jobject)
{

}
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_loadMapNative(JNIEnv*, jobject)
{

}
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_trackingFrameNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
{
	Mat& mGr  = *(Mat*)addrGray;
	Mat& mRgb = *(Mat*)addrRgba;

}


//getpose
JNIEXPORT jboolean JNICALL Java_com_example_ar_ARNativeLib_getGLPoseNative(JNIEnv* env, jobject, jfloatArray arr)
{
	jfloat *carr;
	carr = env->GetFloatArrayElements(arr, false);   //获得Java数组arr的引用的指针
	if(carr == 0) {
		cout<<"get gl pose native ptr is null"<<endl;
		return 0; /* exception occurred */
	}
	//jfloat sum = 0;
	jsize len = (env)->GetArrayLength(arr);
	if(len!=12)
	{
		cout<<"gl pose native len is not 12"<<endl;
		return trackState;
	}
	for(int i=0; i<len; i++) {
		carr[i]=val[i];
		//sum += carr[i];
	}
	env->ReleaseFloatArrayElements(arr, carr, 0);
	return trackState;
}


JNIEXPORT jboolean JNICALL Java_com_example_ar_ARNativeLib_getCenterNative(JNIEnv* env, jobject, jfloatArray arr)
{
	jfloat *carr;
	carr = env->GetFloatArrayElements(arr, false);   //获得Java数组arr的引用的指针
	if(carr == 0) {
		cout<<"get center native ptr is null"<<endl;
		return 0; /* exception occurred */
	}

	//jfloat sum = 0;
	jsize len = (env)->GetArrayLength(arr);
	if(len!=3)
	{
		cout<<"get center native len is not 3"<<endl;
	}
	//cout<<"now we copy center"<<endl;
	//cout<<"before: "<<carr[0]<<"  "<<carr[1]<<"  "<<carr[2]<<"  "<<endl;
	carr[0]=(float)center.x;
	carr[1]=(float)center.y;
	carr[2]=(float)center.z;
	//cout<<"after: "<<carr[0]<<"  "<<carr[1]<<"  "<<carr[2]<<"  "<<endl;
	env->ReleaseFloatArrayElements(arr, carr, 0);
	return trackState;
}


//common
JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_redirectStdOut(JNIEnv*, jobject)
{
	redirectStdOut();
}


JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_processFrameNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
{
	Mat& mGr  = *(Mat*)addrGray;
	Mat& mRgb = *(Mat*)addrRgba;
	vector<KeyPoint> v;

	FastFeatureDetector detector(50);
	detector.detect(mGr, v);
	for( unsigned int i = 0; i < v.size(); i++ )
	{
		const KeyPoint& kp = v[i];
		circle(mRgb, Point(kp.pt.x, kp.pt.y), 10, Scalar(255,0,0,255));
	}
}
void readwrite()
{
	Mat image;
	image = cv::imread("abcd.bmp");
	cv::imwrite("aaaaa.bmp", image);
}

void redirectStdOut()
{
	if(isRedirect==true)
	{
		return;
	}
	freopen("/sdcard/arStdout.txt","w",stdout);
	cout<<"this is ar std out"<<endl;
	isRedirect=true;
}


JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_sendFrameToNative(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
{
	Mat& mGr  = *(Mat*)addrGray;
	Mat& mRgb = *(Mat*)addrRgba;
	vector<KeyPoint> v;

	FastFeatureDetector detector(50);
	detector.detect(mGr, v);
	for( unsigned int i = 0; i < v.size(); i++ )
	{
		const KeyPoint& kp = v[i];
		circle(mRgb, Point(kp.pt.x, kp.pt.y), 10, Scalar(255,0,0,255));
	}
	/*Mat& mGr  = *(Mat*)addrGray;
	Mat& mRgb = *(Mat*)addrRgba;
	Sobel(mRgb,mRgb,mRgb.depth(),1,1);
	//Mat dst;
	//Laplacian(mRgb,dst,mRgb.depth());
	//mRgb=dst.clone();
	return;
	rgblock.lock();
	rgb=mRgb.clone();
	//gray=mGr.clone();
	obtainNewFrame=true;
	rgblock.unlock();

	processRgbLock.lock();
	if(processNewFrame==true)
	{
		mRgb=processRgb.clone();
		processNewFrame=false;
	}
	processRgbLock.unlock();*/
}

void processFrameFirstStageCycle()
{
	while(true)
	{
		LOGE("process frame first stage cycle");
		Mat firstStageFrame;

		/*while(cameraToFirstReady==false)
		{
			//LOGE("not obtain new frame");
			//cameraToFirstLock.unlock();
			usleep(2*1000);
			continue;
		}
		cameraToFirstLock.lock();
		LOGE("finally obtain new frame first");
		firstStageFrame=cameraToFirstRgb.clone();
		cameraToFirstReady=false;

		cameraToFirstLock.unlock();*/
		cameraToFirst.output(firstStageFrame);

		//Mat& dst;
		LOGE("begin edge detection first");
		bool shouldQuit = processFrameFirstStage(firstStageFrame, pipeline, drawingCtx);

		firstToSecond.input(firstStageFrame);

		//cvtColor(dst,dst,CV_GRAY2RGB);
		/*while(firstToSecondReady==true)
		{
			//LOGE("not obtain new frame");
			//cameraToFirstLock.unlock();
			usleep(2*1000);
			continue;
		}
		firstToSecondLock.lock();
		firstToSecondRgb=firstStageFrame.clone();
		firstToSecondReady=true;
		firstToSecondLock.unlock();
		LOGE("begin sleep first");*/
		//sleep(20);
	}
}

void processFrameSecondStageCycle()
{
	while(true)
	{
		LOGE("process frame second stage cycle");
		Mat secondStageFrame;
		Mat secondStageDescriptors;

		/*while(firstToSecondReady==false)
		{
			//LOGE("not obtain new frame");
			//firstToSecondLock.unlock();
			usleep(2*1000);
			continue;
		}
		firstToSecondLock.lock();
		LOGE("finally obtain new frame second");
		secondStageFrame=firstToSecondRgb.clone();
		firstToSecondReady=false;

		firstToSecondLock.unlock();*/

		firstToSecond.output(secondStageFrame);

		//Mat& dst;
		LOGE("begin edge detection second");
		bool shouldQuit = processFrameSecondStage(secondStageFrame, pipeline, drawingCtx,secondStageDescriptors);

		secondToThird.input(secondStageDescriptors);

		//cvtColor(dst,dst,CV_GRAY2RGB);
		/*while(secondToRenderReady==true)
		{
			//LOGE("not obtain new frame");
			//cameraToFirstLock.unlock();
			usleep(2*1000);
			continue;
		}
		secondToRenderLock.lock();
		secondToRenderRgb=secondStageFrame.clone();
		secondToRenderReady=true;
		secondToRenderLock.unlock();
		LOGE("begin sleep second");*/
		//sleep(20);
	}
}


void processFrameThirdStageCycle()
{
	while(true)
	{
		LOGE("process frame third stage cycle");
		Mat thirdStageFrame;
		Mat thirdStageDescriptors;

		secondToThird.output(thirdStageDescriptors);
		//usleep(200*1000);
		//Mat& dst;
		LOGE("begin edge detection third");
		Timer timer;
		timer.start();
		double thirdStageStart=timer.getElapsedTimeInMilliSec();
		bool shouldQuit = processFrameThirdStage(thirdStageFrame, pipeline, drawingCtx,thirdStageDescriptors);
		double thirdStageEnd=timer.getElapsedTimeInMilliSec();
		double thirdStageDuration = thirdStageEnd-thirdStageStart;
		cout<<"third stage duration: "<<thirdStageDuration<<endl;
		thirdToRender.input(thirdStageFrame);


	}


	//}
}


void cycleProcess()
{
	thread t1(processFrameFirstStageCycle);
	thread t2(processFrameSecondStageCycle);
	thread t3(processFrameThirdStageCycle);
	t1.join();
	t2.join();
	t3.join();
	while(true)
	{

		LOGE("edge detection");
		//cameraToFirstReady=false;
		/*Mat currentFrame;
		rgblock.lock();
		if(obtainNewFrame==false)
		{
			LOGE("not obtain new frame");
			rgblock.unlock();
			usleep(10*1000);
			continue;
		}
		currentFrame=rgb.clone();
		obtainNewFrame=false;

		rgblock.unlock();

		//Mat& dst;
		LOGE("begin edge detection");
		bool shouldQuit = processFrameFirstStage(currentFrame, pipeline, drawingCtx);
		shouldQuit=processFrameSecondStage(currentFrame, pipeline, drawingCtx);
		//cvtColor(dst,dst,CV_GRAY2RGB);

		processRgbLock.lock();
		processRgb=currentFrame.clone();
		processNewFrame=true;
		processRgbLock.unlock();
		LOGE("begin sleep");
		//sleep(20);*/
	}
}

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_cycleProcessNative(JNIEnv*, jobject)
{
	cycleProcess();

	/*while(true)
	{
		LOGE("edge detection native");
		usleep(30*1000);
	}*/
}

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_FindFeatures(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
{
	/*Mat& mGr  = *(Mat*)addrGray;
    Mat& mRgb = *(Mat*)addrRgba;
    vector<KeyPoint> v;

    FastFeatureDetector detector(50);
    detector.detect(mGr, v);
    for( unsigned int i = 0; i < v.size(); i++ )
    {
        const KeyPoint& kp = v[i];
        circle(mRgb, Point(kp.pt.x, kp.pt.y), 10, Scalar(255,0,0,255));
    }*/
	//redirectStdOut();
	//map1.findfeature((long long)addrGray,(long long)addrRgba);
	//map1.test();
	usleep(1000*30);
}

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_storeError(JNIEnv*, jobject)
{
	pipeline.m_patternDetector.printError();
	pipeline.m_patternDetector.calcWindowArea();
}

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_showRects(JNIEnv*, jobject)
{
	if(isShowRects==false)
	{
		isShowRects=true;
	}
	else
	{
		isShowRects=false;
	}
}

JNIEXPORT void JNICALL Java_com_example_ar_ARNativeLib_showPoints(JNIEnv*, jobject)
{
	if(isShowPoints==false)
	{
		isShowPoints=true;
	}
	else
	{
		isShowPoints=false;
	}
}
