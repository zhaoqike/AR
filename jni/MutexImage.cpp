/*
 * MutexImage.cpp
 *
 *  Created on: 2014Äê12ÔÂ5ÈÕ
 *      Author: ZhaoQike
 */
#include "MutexImage.h"


void MutexImage::input(Mat& img)
{
	while(ready==true){
		usleep(500);
		continue;
	}
	m.lock();
	image=img.clone();
	ready=true;
	m.unlock();

}

void MutexImage::output(Mat& img)
{
	while(ready==false){
		usleep(500);
		continue;
	}
	m.lock();
	img=image.clone();
	ready=false;
	m.unlock();
}



