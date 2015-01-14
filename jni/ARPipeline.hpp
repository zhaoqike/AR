

#ifndef ARPIPELINE_HPP
#define ARPIPELINE_HPP

////////////////////////////////////////////////////////////////////
// File includes:
#include "PatternDetector.hpp"
#include "CameraCalibration.hpp"
#include "GeometryTypes.hpp"



class ARPipeline
{
public:
	ARPipeline();
  ARPipeline(const Mat& patternImage, const CameraCalibration& calibration);

  void init(const Mat& patternImage, const CameraCalibration& calibration);
  void init(const vector<Mat>& patternImageList, const CameraCalibration& calibration);


  bool processFrameFirstStage(Mat& inputFrame);
  bool processFrameSecondStage(Mat& inputFrame,Mat& descriptors);
  bool processFrameThirdStage(Mat& inputFrame,Mat& descriptors);
  bool processFrame(Mat& inputFrame);

  const Transformation& getPatternLocation() const;

  PatternDetector     m_patternDetector;
private:

private:
  CameraCalibration   m_calibration;
  Pattern             m_pattern;
  PatternTrackingInfo m_patternInfo;
  vector<Pattern> m_patternList;
  //PatternDetector     m_patternDetector;
};

#endif
