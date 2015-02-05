#include "TrackerTimer.h"


TrackerTimer::TrackerTimer()
{
	detect = 0;
	extract = 0;
	matchkf = 0;
	matchpt = 0;
	ransac = 0;
}


TrackerTimer::~TrackerTimer()
{
}

ostream& operator << (ostream& os, TrackerTimer& t) //定义运算符“<<”重载函数
{
	os << t.detect << ',' << t.extract << ',' << t.matchkf << ',' << t.matchpt << ',' << t.ransac;
	return os;
}
