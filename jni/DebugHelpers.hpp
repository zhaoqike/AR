

#ifndef DEBUG_HELPERS_HPP
#define DEBUG_HELPERS_HPP

#include <string>
#include <sstream>
using namespace std;
using namespace cv;

// Does lexical cast of the input argument to string
template <typename T>
string ToString(const T& value)
{
    ostringstream stream;
    stream << value;
    return stream.str();
}

namespace cv
{
    // This function used to show and save the image to the disk (used for during chapter writing).
    inline void showAndSave(string name, const Mat& m)
    {
        imshow(name, m);
        imwrite(name + ".png", m);
        //waitKey(25);
    }

    // Draw matches between two images
    inline Mat getMatchesImage(Mat query, Mat pattern, const vector<KeyPoint>& queryKp, const vector<KeyPoint>& trainKp, vector<DMatch> matches, int maxMatchesDrawn)
    {
        Mat outImg;

        if (matches.size() > maxMatchesDrawn)
        {
            matches.resize(maxMatchesDrawn);
        }

        drawMatches
            (
            query, 
            queryKp, 
            pattern, 
            trainKp,
            matches, 
            outImg, 
            Scalar(0,200,0,255), 
            Scalar::all(-1),
            vector<char>(), 
            DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
            );

        return outImg;
    }
}

#endif
