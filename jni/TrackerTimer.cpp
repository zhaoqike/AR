#include "TrackerTimer.h"


TrackerTimer::TrackerTimer()
{
}


TrackerTimer::~TrackerTimer()
{
}

ostream& operator << (ostream& os, TrackerTimer& t) //�����������<<�����غ���
{
	os << t.detect << ' ' << t.extract << ' ' << t.matchkf << ' ' << t.matchpt << ' ' << t.ransac << endl;
	return os;
}
