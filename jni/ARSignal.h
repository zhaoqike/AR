#pragma once
#include <vector>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

enum SignalTag
{
	printWarp,
	printError,
	printDistance
};


class ARSignal
{
public:
	ARSignal();
	~ARSignal();


	SignalTag tag;


	static vector<int> signalList;
	static void putSignal(SignalTag t);
	static bool existSignal(SignalTag t);
	static void eatOneSignal(SignalTag t);
	static void eatAllSignal(SignalTag t);

	static void processSignal(ARSignal signal);
	static void processWarpSignal(ARSignal signal);
};

