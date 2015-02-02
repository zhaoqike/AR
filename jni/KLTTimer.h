#pragma once
#include "ARTimer.h"
class KLTTimer:public ARTimer
{
public:
	KLTTimer();
	~KLTTimer();
	float klt;
	float ransac;
	float store;
};

