#pragma once
#include "ARPipeline.hpp"
#include "ARDrawingContext.hpp"
#include "Utils.h"
#include "DebugHelpers.hpp"

class AREngine
{
public:
	AREngine();
	~AREngine();
	void init();
	bool processFrame(Mat& cameraFrame);


	ARPipeline pipeline;
	ARDrawingContext drawingCtx;
	Eye eye;
};

