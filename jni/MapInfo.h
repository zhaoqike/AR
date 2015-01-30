#pragma once

#include <string>
#include <vector>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;


class MapInfo
{
public:
	MapInfo();
	~MapInfo();


	string imagePath;

	vector<Point2f> pointList;

	vector<string> modelPathList;
};

