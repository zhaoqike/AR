
////////////////////////////////////////////////////////////////////
// File includes:
#include "ARPipeline.hpp"
#include "Timer.h"
#include "DebugPrint.h"

ARPipeline::ARPipeline() {

}

ARPipeline::ARPipeline(const Mat& patternImage, const CameraCalibration& calibration) :m_calibration(calibration) {
	m_patternDetector.buildPatternFromImage(patternImage, m_pattern);
	m_patternDetector.train(m_pattern);
}

void ARPipeline::init(const Mat& patternImage, const CameraCalibration& calibration) {
	m_calibration = calibration;
	conprint << "begin build pattern from image" << endl;
	m_patternDetector.buildPatternFromImage(patternImage, m_pattern);
	conprint << "end build pattern from image" << endl;
	m_patternDetector.train(m_pattern);
}

void ARPipeline::init(const vector<Mat>& patternImageList, const CameraCalibration& calibration) {
	m_calibration = calibration;
	for (int i = 0; i < patternImageList.size(); i++) {
		m_patternDetector.buildPatternFromImage(patternImageList[i], m_patternList[i]);
	}
	m_patternDetector.trainPatternList(m_patternList);
}

bool ARPipeline::processFrame(Mat& inputFrame) {
	Timer timer;
	timer.start();
	double findPatternStart = timer.getElapsedTimeInMilliSec();
	bool patternFound = m_patternDetector.findPattern(inputFrame, m_patternInfo);
	double findPatternEnd = timer.getElapsedTimeInMilliSec();
	double findPatternDuration = findPatternEnd - findPatternStart;
	conprint << "find pattern uses : " << findPatternDuration << endl;

	if (patternFound) {
		m_patternInfo.computePose(m_pattern, m_calibration);
	}

	return patternFound;
}

bool ARPipeline::processFrameFirstStage(Mat& inputFrame) {
	bool patternFound = m_patternDetector.findPatternFirstStage(inputFrame, m_patternInfo);
	return patternFound;
}

bool ARPipeline::processFrameSecondStage(Mat& inputFrame, Mat& descriptors) {
	bool patternFound = m_patternDetector.findPatternSecondStage(inputFrame, m_patternInfo, descriptors);

	return patternFound;
}

bool ARPipeline::processFrameThirdStage(Mat& inputFrame, Mat& descriptors) {
	bool patternFound = m_patternDetector.findPatternThirdStage(inputFrame, m_patternInfo, descriptors);

	if (patternFound) {
		m_patternInfo.computePose(m_pattern, m_calibration);
	}

	return patternFound;
}

const Transformation& ARPipeline::getPatternLocation() const {
	return m_patternInfo.pose3d;
}
