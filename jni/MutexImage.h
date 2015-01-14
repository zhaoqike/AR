/*
 * MutexImage.h
 *
 *  Created on: 2014Äê12ÔÂ5ÈÕ
 *      Author: ZhaoQike
 */

#ifndef MUTEXIMAGE_H_
#define MUTEXIMAGE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>
#include <unistd.h>


using namespace std;
using namespace cv;

class MutexImage
{
public:
	mutex m;
	bool ready;
	Mat image;

	void input(Mat& img);


	void output(Mat& img);

};


#endif /* MUTEXIMAGE_H_ */
