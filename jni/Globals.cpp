#include "Globals.h"


bool isShowRects = true;
bool isShowPoints = true;
bool isShowTexts = true;
bool isMultiScale = true;



ARPipeline pipeline;
ARDrawingContext drawingCtx;



CameraCalibration calibration(526.58037684199849f, 524.65577209994706f, 318.41744018680112f, 202.96659047014398f);


Mesh *g_pMesh = NULL; // not necessary, but a nice CYA habit
PMesh *g_pProgMesh = NULL;


GLuint *gTexture = 0;

ARDrawing drawing;
ARError arerror;
