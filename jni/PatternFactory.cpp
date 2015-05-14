#include "PatternFactory.h"
#include "Bj1Info.h"
#include "Zg1Info.h"


PatternFactory::PatternFactory()
{
}


PatternFactory::~PatternFactory()
{
}

void PatternFactory::buildPattern(Type t)
{
	switch (t)
	{
	case bj1:
	{
				Bj1Info bj1Info;
				imagePath = bj1Info.imagePath;
				blendImagePath = bj1Info.blendImagePath;

				pointList = bj1Info.pointList;

				modelPathList = bj1Info.modelPathList;
				break;
	}
	case zg1:
	{
				Zg1Info zg1Info;
				imagePath = zg1Info.imagePath;
				blendImagePath = zg1Info.blendImagePath;

				pointList = zg1Info.pointList;

				modelPathList = zg1Info.modelPathList;
				break;
	}
	}
}
