#include "KLTTimer.h"


KLTTimer::KLTTimer()
{
}


KLTTimer::~KLTTimer()
{
}

ostream& operator << (ostream& os, KLTTimer& t) //定义运算符“<<”重载函数
{
	os << t.klt << ' ' << t.ransac << ' ' << t.store << endl;
	return os;
}