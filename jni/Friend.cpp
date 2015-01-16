#include "Friend.h"


Friend::Friend()
{
}


Friend::~Friend()
{
}

int Friend::getFrameNum(PatternDetector& pd)
{
	return pd.m_lostFrameNum;
}