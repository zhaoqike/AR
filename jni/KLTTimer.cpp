#include "KLTTimer.h"


KLTTimer::KLTTimer()
{
}


KLTTimer::~KLTTimer()
{
}

ostream& operator << (ostream& os, KLTTimer& t) //�����������<<�����غ���
{
	os << t.klt << ' ' << t.ransac << ' ' << t.store << endl;
	return os;
}