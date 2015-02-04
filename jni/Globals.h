#pragma once
#include "ARDrawingContext.hpp"
#include "ARPipeline.hpp"
#include "Model/pmesh.h"
#include "ARDrawing.h"
#include "ARError.h"
#include "AREngine.h"
#include "Signal.h"
#include "ARTimer.h"
#include "TrackerTimer.h"
#include "KLTTimer.h"
#include "DebugPrint.h"


#ifndef WIN32
#include <jni.h>
#include <android/log.h>
#include <GLES/gl.h>
#include <GLES/glext.h>
#define  LOG_TAG    "libgljni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#else
#include <Windows.h>
#include <gl/gl.h>
#include <gl/glu.h>
#endif

PMesh* makeMesh(string path);
struct Model
{
	//int meshIndex;
	int edgeNum;
	Point2f p2d;
	Point3f p3d;
	int level = 0;
	Model(int index);
	Model(string path);
	Model();
	int meshEdgeNum();
	int getEdgeNum(float distance);
	int getActiveNum();
	PMesh* pmesh;
};

extern vector<Model> kfmodels;
extern vector<Point2f> pointList;

extern bool isShowRects;
extern bool isShowPoints;
extern bool isShowTexts;
extern bool isMultiScale;
extern bool isWarp;
extern bool isPoly;
extern bool isOpticalFlow;
extern bool isDrawModel;
extern bool isPrint;
extern bool isLod;

//extern bool isPrint;
//#define conprint if(isPrint) cout




extern ARPipeline pipeline;
extern ARDrawingContext drawingCtx;
extern CameraCalibration calibration;






//extern PMesh *g_pProgMesh;
//extern Mesh *g_pMesh;


extern GLuint *gTexture;


const int screenWidth = 640;
const int screenHeight = 480;

const int updateFrameNum = 20;

const int maxLostFrames = 5;


const int REDUCE_TRI_PERCENT = 5;
const int NUM_PAGEUPDN_INTERVALS = 100 / REDUCE_TRI_PERCENT;

extern ARDrawing drawing;
extern ARError arerror;


//extern vector<PMesh* > pmeshList;
extern string imagePath;
extern vector<string> modelPathList;

extern vector<Eye> eyes;

const int oriIndex = 0;

const float MAX_DISTANCE = 2.5;
const float MIN_DISTANCE = 1.0;

const float MAX_EDGE = 1.0;
const float MIN_EDGE = 0.4;

extern AREngine engine;

//extern vector<Signal> signalList;

extern bool isPrintWarp;
extern bool isPrintTime;


void changeEdgeNum(Model& model, int newNum);
