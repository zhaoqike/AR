#pragma once
#include "ARTimer.h"
#include <iostream>

using namespace std;
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


	friend ostream& operator<<(ostream& os, TrackerTimer& t);
};

