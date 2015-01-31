#include "ARSignal.h"


//init signal list
vector<int> ARSignal::signalList(100, 0);

ARSignal::ARSignal()
{
}


ARSignal::~ARSignal()
{
}


void ARSignal::putSignal(SignalTag t)
{
	signalList[t]++;
}

bool ARSignal::existSignal(SignalTag t)
{
	return signalList[t] > 0;
}

void ARSignal::eatOneSignal(SignalTag t)
{
	if (signalList[t] > 0)
	{
		signalList[t]--;
	}
}
void ARSignal::eatAllSignal(SignalTag t)
{
	if (signalList[t] > 0)
	{
		signalList[t] = 0;
	}
}

void ARSignal::processSignal(ARSignal signal)
{
	SignalTag t = signal.tag;
	if (existSignal(t))
	{
		switch (t)
		{
		case printWarp:
			processWarpSignal(signal);
			break;
		}
	}
	eatOneSignal(t);
}

void ARSignal::processWarpSignal(ARSignal signal)
{
	/*string warpedPath = "/sdcard/";
	string oriName = warpedPath + "ori.jpg";
	string warpedName = warpedPath + "warped.jpg";
	imwrite(oriName, ori);
	imwrite(warpedName, warped);
	eatOneSignal(printWarp);
	SignalTag*/
}