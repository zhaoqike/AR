#include "Globals.h"


bool isShowRects = true;
bool isShowPoints = true;
bool isShowTexts = true;
bool isMultiScale = false;



ARPipeline pipeline;
ARDrawingContext drawingCtx;




CameraCalibration calibration(640.0f, 640.0f, 320.0f, 240.0f);


//Mesh *g_pMesh = NULL; // not necessary, but a nice CYA habit
//PMesh *g_pProgMesh = NULL;


GLuint *gTexture = 0;

ARDrawing drawing;
ARError arerror;

vector<PMesh* > pmeshList;

vector<string> modelPathList={
		"/sdcard/models/apple.ply",
		"/sdcard/models/hind.ply",
		"/sdcard/models/AIRBOAT.ply",
		"/sdcard/models/big_porsche.ply",
		"/sdcard/models/CESSNA.ply",
		"/sdcard/models/cow.ply",
		"/sdcard/models/FLAMINGO.ply",
		"/sdcard/models/hind.ply",
		"/sdcard/models/PORSCHE.ply",
		"/sdcard/models/SHUTTLE.ply",
};
