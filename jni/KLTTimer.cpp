#include "KLTTimer.h"


KLTTimer::KLTTimer()
{
	klt = 0;
	ransac = 0;
	store = 0;
}


KLTTimer::~KLTTimer()
{
}

ostream& operator << (ostream& os, KLTTimer& t) //定义运算符“<<”重载函数
{
	os << t.klt << ',' << t.ransac << ',' << t.store;
	return os;
}