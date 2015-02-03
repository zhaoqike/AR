#include "Bj1Info.h"


Bj1Info::Bj1Info()
{
	imagePath = "/sdcard/ar/bj1/p4.jpg";

	pointList = {
		Point2f(1341, 812), //tiananmen gugong
		Point2f(730, 81),//yuanmingyuan
		Point2f(907, 141), //qinghuadaxue
		Point2f(1064, 549)//beijingbeizhan

	};

	modelPathList = {
		"/sdcard/ar/bj1/models/apple.ply",
		"/sdcard/ar/bj1/models/hind.ply",
		"/sdcard/ar/bj1/models/AIRBOAT.ply",
		"/sdcard/ar/bj1/models/big_porsche.ply",
		"/sdcard/ar/bj1/models/CESSNA.ply",
		"/sdcard/ar/bj1/models/cow.ply",
		"/sdcard/ar/bj1/models/FLAMINGO.ply",
		"/sdcard/ar/bj1/models/hind.ply",
		"/sdcard/ar/bj1/models/PORSCHE.ply",
		"/sdcard/ar/bj1/models/SHUTTLE.ply"
	};
}


Bj1Info::~Bj1Info()
{
}
