#include "TrackerTimer.h"


TrackerTimer::TrackerTimer()
{
}


TrackerTimer::~TrackerTimer()
{
}

ostream& operator << (ostream& os, TrackerTimer& t) //定义运算符“<<”重载函数
{
	os << t.detect << ' ' << t.extract << ' ' << t.matchkf << ' ' << t.matchpt << ' ' << t.ransac << endl;
	return os;
}
