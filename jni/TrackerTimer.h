#pragma once
#include "ARTimer.h"
class TrackerTimer:public ARTimer
{
public:
	TrackerTimer();
	~TrackerTimer();

	float detect;
	float extract;
	float matchkf;
	float matchpt;
	float ransac;

};

