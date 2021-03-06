

#ifndef EXAMPLE_MARKERLESS_AR_PATTERNDETECTOR_HPP
#define EXAMPLE_MARKERLESS_AR_PATTERNDETECTOR_HPP

////////////////////////////////////////////////////////////////////
// File includes:
#include "Pattern.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>

using namespace std;
using namespace cv;

struct MatchState
{
	bool found;
	int num;
	int index;
};

struct AlphaState
{
	double alpha;
	int index;
};

class PatternDetector
{
public:
	/**
	 * Initialize a pattern detector with specified feature detector, descriptor extraction and matching algorithm
	 */
	PatternDetector
	(
			Ptr<FeatureDetector>     detector  = new FastFeatureDetector,
			Ptr<DescriptorExtractor> extractor = new BriefDescriptorExtractor,
			Ptr<DescriptorMatcher>   matcher   = new BFMatcher(NORM_HAMMING, true),


			//Ptr<DescriptorMatcher>   matcher   = new FlannBasedMatcher(),


			//Ptr<FeatureDetector>     detector  = new ORB(1000),
			//Ptr<DescriptorExtractor> extractor = new FREAK(false, false),
			//Ptr<DescriptorMatcher>   matcher   = new BFMatcher(NORM_HAMMING, true),
			bool enableRatioTest= false,
			bool enableWrap=true,
			bool enableOpticalFlow=true,
			bool estimatedHomoFound=false
	);

	int minNum=10;
	int indexCount = 3;

	/**
	 *
	 */
	void train(const Pattern& pattern);
	void trainPatternList(const vector<Pattern>& pattern);
	int findKeyframe();


	double getWindowDivPictureScale(int width, int height);
	double getPictureDivWindowScale(int width, int height);
	int getLayerNum(int width, int height);
	void cutImage(Mat& image, int level, vector<Rect>& rectList);

	/**
	 * Initialize Pattern structure from the input image.
	 * This function finds the feature points and extract descriptors for them.
	 */
	void makeKeyFrame(const Mat& image, Rect& range, Size& oriSize, KeyFrame& keyframe);
	void makeKeyFrame(const Mat& oriimage, Rect& range,KeyFrame& kf);
	void makeKeyFrameList(const Mat& img, vector<KeyFrame>& keyFrameList);
	void makeKeyFrameList(const Mat& img, Layer& layer, int level);
	void buildPatternFromImage(const Mat& image, Pattern& pattern);

	bool findPatternFirstStage(Mat& image, PatternTrackingInfo& info);
	bool findPatternSecondStage(Mat& image, PatternTrackingInfo& info, Mat& descriptors);
	bool findPatternThirdStage(Mat& image, PatternTrackingInfo& info, Mat& descriptors);
	/**
	 * Tries to find a @pattern object on given @image.
	 * The function returns true if succeeded and store the result (pattern 2d location, homography) in @info.
	 */
	bool findPattern(Mat& image, PatternTrackingInfo& info);
	bool needNewPoints();
	int matchKeyFrames(Mat& homography, vector<int>& indexes, vector<int>& matchIdxes, vector<int>& estiIdxes, string& str);
	MatchState matchKeyFrame(int index, vector<DMatch>& matches);
	//bool findPatternTwice(Mat& image, PatternTrackingInfo& info);

	double calcWindowArea();

	bool estimatedHomographyFound;
	bool enableRatioTest;
	bool enableWrap;
	bool enableOpticalFlow;
	float homographyReprojectionThreshold;

//protected:

	static bool compareAlpha(AlphaState a, AlphaState b);
	static bool compareCount(MatchState a, MatchState b);

	bool extractFeatures(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors) const;

	void getMatches(const Mat& queryDescriptors, vector<DMatch>& matches);

	void getMatches(const Mat& queryDescriptors, int index, vector<DMatch>& matches);
	void getMatches(const Mat& queryDescriptors, vector<int>& indexes, vector<DMatch>& matches);

	void drawContours(Mat& image, PatternTrackingInfo& info);
	void drawContours(Mat& image, PatternTrackingInfo& info, vector<int> indexes);
	void draw2Contours(Mat& image, PatternTrackingInfo& info, vector<int> matchIndexes, vector<int> estiIndexes);

	/**
	 * Get the gray image from the input image.
	 * Function performs necessary color conversion if necessary
	 * Supported input images types - 1 channel (no conversion is done), 3 channels (assuming BGR) and 4 channels (assuming BGRA).
	 */
	static void getGray(const Mat& image, Mat& gray);
	float point_distance(Point2f& p1, Point2f& p2);
	float computeError(Mat homography);
	void printError();

	/**
	 *
	 */
	static bool refineMatchesWithHomography(
			const vector<KeyPoint>& queryKeypoints,
			const vector<KeyPoint>& trainKeypoints,
			float reprojectionThreshold,
			vector<DMatch>& matches,
			Mat& homography);

//private:
	vector<KeyPoint> m_queryKeypoints;
	Mat                   m_queryDescriptors;
	vector<DMatch>   m_matches;
	vector< vector<DMatch> > m_knnMatches;

	Mat                   m_grayImg;

	//optical flow
	Mat  				m_grayImgPrev;
	vector<Point2f> points[2]; // tracked features from 0->1
	vector<Point2f> before;
	vector<Point2f> after;
	vector<Point2f> initial;   // initial position of tracked points
	vector<Point2f> patternPoints;
	vector<Point2f> features;  // detected features
	int max_count;	  // maximum number of features to detect
	double qlevel;    // quality level for feature detection
	double minDist;   // minimum distance between two feature points
	vector<uchar> status; // status of tracked features
	vector<float> err;    // error in tracking
	//int keyframeIndex;


	// determine which tracked point should be accepted
	bool acceptTrackedPoint(int i);
	// handle the currently tracked points
	//void handleTrackedPoints( Mat &frame,  Mat &output);
	void handleTrackedPoints( Mat &frame,  Mat &output, Scalar scalar=Scalar(255,0,0) );

	void makeLayerList(const Mat& image, vector<Layer>& layerList, int layers);




	mutex 				m_firstToSecondMutex;
	Mat 				m_firstToSecondImg;
	Mat 				m_firstStageImg;
	Mat					m_secondStageImg;

	Mat                   m_warpedImg;
	Mat                   m_roughHomography;
	Mat                   m_refinedHomography;

	Mat 				m_estimatedHomography;
	Mat 				m_initialHomography;

	Pattern                  m_pattern;
	vector<Pattern> 		m_patternList;
	Ptr<FeatureDetector>     m_detector;
	Ptr<DescriptorExtractor> m_extractor;
	Ptr<DescriptorMatcher>   m_matcher;

	vector<float> errs;

	int m_lostFrameNum;
	int m_opticalFrameNum;
};

#endif
