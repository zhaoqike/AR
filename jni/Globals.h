#include "ARDrawingContext.hpp"
#include "ARPipeline.hpp"
#include "Model/pmesh.h"
#include "ARDrawing.h"
#include "ARError.h"


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

extern bool isShowRects;
extern bool isShowPoints;
extern bool isShowTexts;
extern bool isMultiScale;



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

extern ARDrawing drawing;
extern ARError arerror;


extern vector<PMesh* > pmeshList;
extern vector<string> modelPathList;
