#include "PatternFactory.h"
#include "Bj1Info.h"


PatternFactory::PatternFactory()
{
}


PatternFactory::~PatternFactory()
{
}

void PatternFactory::buildPattern(Type t)
{
	if (t == bj1)
	{
		Bj1Info bj1Info;
		imagePath=bj1Info.imagePath;


		pointList=bj1Info.pointList;

		modelPathList=bj1Info.modelPathList;
	}
}
