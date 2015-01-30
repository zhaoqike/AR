#pragma once
#include <string>
#include <vector>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;
class Bj1Info
{
public:
	Bj1Info();
	~Bj1Info();


	const string imagePath = "/sdcard/ar/bj1/bj1.jpg";

	const vector<Point2f> pointList = {
		Point2f(1341, 812), //tiananmen gugong
		Point2f(730, 81),//yuanmingyuan
		Point2f(907, 141), //qinghuadaxue
		Point2f(1064, 549),//beijingbeizhan

	};

	const vector<string> modelPathList = {
		"/sdcard/ar/bj1/models/apple.ply",
		"/sdcard/ar/bj1/models/hind.ply",
		"/sdcard/ar/bj1/models/AIRBOAT.ply",
		"/sdcard/ar/bj1/models/big_porsche.ply",
		"/sdcard/ar/bj1/models/CESSNA.ply",
		"/sdcard/ar/bj1/models/cow.ply",
		"/sdcard/ar/bj1/models/FLAMINGO.ply",
		"/sdcard/ar/bj1/models/hind.ply",
		"/sdcard/ar/bj1/models/PORSCHE.ply",
		"/sdcard/ar/bj1/models/SHUTTLE.ply",
	};
};

