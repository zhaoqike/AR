#pragma once
#include "ARTimer.h"
#include <iostream>
using namespace std;
class KLTTimer:public ARTimer
{
public:
	KLTTimer();
	~KLTTimer();
	float klt;
	float ransac;
	float store;

	friend ostream& operator<<(ostream& os, KLTTimer& t);
};

