#include "ARError.h"


ARError::ARError()
{
}


ARError::~ARError()
{
}

float ARError::point_distance(Point2f& p1, Point2f& p2)
{
	float x = p1.x - p2.x;
	float y = p1.y - p2.y;
	return sqrtf(x*x + y*y);
}

float ARError::computeError(PatternDetector& pd, Mat homography)
{
	if (homography.rows != 3 || homography.cols != 3)
	{
		return 0;
	}
	vector<Point2f> persPoints;
	perspectiveTransform(pd.patternPoints, persPoints, homography);
	if (persPoints.size() != pd.after.size())
	{
		cout << "compute error size not match";
		return 0;
	}
	float err = 0;
	for (int i = 0; i < persPoints.size(); i++)
	{
		err += (point_distance(persPoints[i], pd.after[i]));
	}
	/*for (int i = 0; i < persPoints.size(); i++)
	{
	err += (point_distance(persPoints[i], after[i]));
	}*/
	err /= persPoints.size();
	errs.push_back(err);
	return err;
}

void ARError::printError()
{
	fstream errfile;
	errfile.open("sdcard/err.txt", ios::out);
	errfile << "errors" << endl;
	errfile << errs.size() << endl;
	for (int i = 0; i < errs.size(); i++)
	{
		errfile << errs[i] << endl;
	}
	errfile.flush();
	errfile.close();
}