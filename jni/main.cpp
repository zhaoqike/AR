


////////////////////////////////////////////////////////////////////
// File includes:
#include "ARDrawingContext.hpp"
#include "ARPipeline.hpp"
#include "DebugHelpers.hpp"

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <gl/gl.h>
#include <gl/glu.h>

#include <ctime>
#include "Timer.h"
#include "Model/pmesh.h"
#include "Model/mesh.h"

#include "Globals.h"


using namespace std;
using namespace cv;



//extern GLuint *gTexture;
//GLuint *gTexture = 0;

//extern PMesh *g_pProgMesh;



//Mesh *g_pMesh = NULL; // not necessary, but a nice CYA habit
//PMesh *g_pProgMesh = NULL;


//extern ARPipeline pipeline;
//extern ARDrawingContext drawingCtx;
extern CameraCalibration calibration;
extern AREngine engine;
//ARPipeline pipeline;
//ARDrawingContext drawingCtx;


extern bool isMultiScale;

bool trackState=false;
//bool isMultiScale=true;

#ifndef WIN32 
#include <jni.h>
#include <android/log.h>
#define  LOG_TAG    "libgljni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#endif
/**
 * Processes a recorded video or live view from web-camera and allows you to adjust homography refinement and 
 * reprojection threshold in runtime.
 */
void processVideo(Mat& patternImage, CameraCalibration& calibration, VideoCapture& capture);

/**
 * Processes single image. The processing goes in a loop.
 * It allows you to control the detection process by adjusting homography refinement switch and 
 * reprojection threshold in runtime.
 */
void processSingleImage(Mat& patternImage, CameraCalibration& calibration, Mat& image);

/**
 * Performs full detection routine on camera frame and draws the scene using drawing context.
 * In addition, this function draw overlay with debug information on top of the AR window.
 * Returns true if processing loop should be stopped; otherwise - false.
 */
bool processFrame(Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx);
//bool processFramefuck(Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx);

int main(int argc, const char * argv[])
{
    cout<<"hello"<<endl;


	time_t filetamp;
	filetamp=time(NULL);
	stringstream ss;
	string str;
	ss<<filetamp;
	ss>>str;

	string filename="out"+str+".txt";

	freopen(filename.c_str(),"w",stdout);


	engine.init();
	VideoCapture cap;
	bool opened = cap.open(0);
	if (!opened)
	{
		cout << "camera not open" << endl;
		return 0;
		
	}
	Mat currentFrame;
	cap >> currentFrame;
	while (true)
	{
		engine.processFrame(currentFrame);
		waitKey(5);
	}

	//Mat patternImage = imread("PyramidPattern.jpg");
	//Mat testImage = imread("PyramidPatternTest.bmp");
	//if (!testImage.empty())
	//{
	//	//imshow("win1", patternImage);
	//	//imshow("win2", testImage);
	//	//waitKey(0);
	//	cout << "==================================================================================main begin process" << endl;
	//	processSingleImage(patternImage, calibration, testImage);
	//}
	//return 0;
    /*if (argc < 2)
    {
        cout << "Input image not specified" << endl;
        cout << "Usage: markerless_ar_demo <pattern image> [filepath to recorded video or image]" << endl;
        return 1;
    }*/

    // Try to read the pattern:
    Mat patternImage = imread("book.jpg");
    if (patternImage.empty())
    {
        cout << "Input image cannot be read" << endl;
		cerr << "Input image cannot be read" << endl;
		getchar();
        return 2;
    }
	//Mat testImage = imread("PyramidPatternTest.bmp");
//	VideoCapture cap;
//	bool opened=cap.open(0);
    if (opened)
    {
        processVideo(patternImage, calibration, cap);
    }
	else
	{
		cerr<<"cant open camera"<<endl;
	}
	//processSingleImage(patternImage, calibration, testImage);
	getchar();
	return 0;

    if (argc == 2)
    {
        processVideo(patternImage, calibration, VideoCapture());
    }
    else if (argc == 3)
    {
        string input = argv[2];
        Mat testImage = imread(input);
        if (!testImage.empty())
        {
            processSingleImage(patternImage, calibration, testImage);
        }
        else 
        {
            VideoCapture cap;
            if (cap.open(1))
            {
                processVideo(patternImage, calibration, cap);
            }
        }
    }
    else
    {
        cerr << "Invalid number of arguments passed" << endl;
        return 1;
    }

    return 0;
}

void processVideo(Mat& patternImage, CameraCalibration& calibration, VideoCapture& capture)
{
    // Grab first frame to get the frame dimensions
    Mat currentFrame;  
    capture >> currentFrame;

    // Check the capture succeeded:
    if (currentFrame.empty())
    {
        cout << "Cannot open video capture device" << endl;
        return;
    }

    Size frameSize(currentFrame.cols, currentFrame.rows);

    //ARPipeline pipeline(patternImage, calibration);
    //ARDrawingContext drawingCtx("Markerless AR", frameSize, calibration);
	pipeline.init(patternImage, calibration);
	drawingCtx.init("Markerless AR", frameSize, calibration);


    bool shouldQuit = false;
	Timer t;
	t.start();
	
    do
    {
		double d=t.getElapsedTimeInMilliSec();
		double cvd = (double)cvGetTickCount();
        capture >> currentFrame;
		cout<<"frame"<<endl;
		//char key = (char)waitKey(1);
        if (currentFrame.empty())
        {
            shouldQuit = true;
            continue;
        }
		double d1=t.getElapsedTimeInMilliSec();
		double cvd1 = (double)cvGetTickCount();
        shouldQuit = processFrame(currentFrame, pipeline, drawingCtx);
		double d2=t.getElapsedTimeInMilliSec();
		cout<<d1-d<<"  "<<d2-d1<<"frame "<<1000.0/(d2-d)<<endl;
		cout<<"run time "<< (cvd1-cvd)/(cvGetTickFrequency()*1000)<<endl ;
    } while (!shouldQuit);
}

void processSingleImage(Mat& patternImage, CameraCalibration& calibration, Mat& image)
{
    Size frameSize(image.cols, image.rows);
	//ARPipeline pipeline(patternImage, calibration);
	//ARDrawingContext drawingCtx("Markerless AR", frameSize, calibration);
    pipeline.init(patternImage, calibration);
    drawingCtx.init("Markerless AR", frameSize, calibration);
	/*do
	{
		drawingCtx.updateBackground(patternImage);
		drawingCtx.updateWindow();
		// Read the keyboard input:
		int keyCode = waitKey(5);
	} while (true);*/

    bool shouldQuit = false;
    do
    {
        shouldQuit = processFrame(image, pipeline, drawingCtx);
	} while (!shouldQuit);
}

bool processFrame(Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx)
{
    // Clone image used for background (we will draw overlay on it)
	cout<<"begin clone"<<endl;
    Mat img = cameraFrame.clone();
    // Draw information:
    

    putText(img, "RANSAC threshold: " + ToString(pipeline.m_patternDetector.homographyReprojectionThreshold) + "( Use'-'/'+' to adjust)", Point(10, 30), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0,200,0));

    // Set a new camera frame:
	/*cout << "win1" << endl;
	imshow("win", img);
	waitKey(0);*/
    cout<<"begin update background"<<endl;
    drawingCtx.updateBackground(img);
	//imshow("image",img);
	//waitKey(0);
    // Find a pattern and update it's detection status:
	Mat processImage = cameraFrame.clone();
    drawingCtx.isPatternPresent = pipeline.processFrame(img);
	//putText(img, "fuck", Point(10, 50), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 200, 0));
	drawingCtx.updateBackground(img);
    // Update a pattern pose:
    drawingCtx.patternPose = pipeline.getPatternLocation();

    // Request redraw of the window:
    drawingCtx.updateWindow();
	cout << "end update window" << endl;
	/*cout << "win3" << endl;
	imshow("win3", drawingCtx.m_backgroundImage);
	waitKey(0);*/

    // Read the keyboard input:
    int keyCode = waitKey(5);

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
        //pipeline.m_patternDetector.enableHomographyRefinement = !pipeline.m_patternDetector.enableHomographyRefinement;
		cout<<"change refinement"<<endl;
    }
    else if (keyCode == 27 || keyCode == 'q')
    {
        shouldQuit = true;
    }*/

    return shouldQuit;
}

/*bool processFramefuck(Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx)
{
	// Clone image used for background (we will draw overlay on it)
	cout << "begin clone" << endl;
	Mat img = cameraFrame.clone();

	drawingCtx.updateBackground(img);
	Mat processImage = cameraFrame.clone();
	//cvtColor(processImage, processImage, CV_BGR2GRAY);
	pipeline.processFrame(img);
	drawingCtx.isPatternPresent = true;
	drawingCtx.updateWindow();


	// Read the keyboard input:
	int keyCode = waitKey(5);

	bool shouldQuit = true;

	return shouldQuit;
}*/


