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

ostream& operator << (ostream& os, KLTTimer& t) //�����������<<�����غ���
{
	os << t.klt << ',' << t.ransac << ',' << t.store;
	return os;
}