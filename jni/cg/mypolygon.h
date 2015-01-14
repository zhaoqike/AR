#ifndef POLYGON_H
#define POLYGON_H
#include <vector>
#include "mypoint.h"
#include "myline.h"

using namespace std;

class MyPolygon
{
public:
    MyPolygon();
    MyPolygon(vector<MyPoint> pList);
    ~MyPolygon();
    bool intersect(MyPolygon& a,MyPolygon& b,MyPolygon& interPoly);
    bool intersect(vector<MyPoint>& poly1,vector<MyPoint>& poly2,vector<MyPoint>& interPoly);
    //void polygonToPLineList(vector<Polygon>& polyList, vector<CPline>& plineList);
    //void polygonToPLine(Polygon& polyList, CPline& plineList);

    //鎺掓枼瀹為獙
    bool IsRectCross(const MyPoint &p1,const MyPoint &p2,const MyPoint &q1,const MyPoint &q2);
    //璺ㄧ珛鍒ゆ柇
    bool IsLineSegmentCross(const MyPoint &pFirst1,const MyPoint &pFirst2,const MyPoint &pSecond1,const MyPoint &pSecond2);
    bool GetCrossPoint(const MyPoint &p1,const MyPoint &p2,const MyPoint &q1,const MyPoint &q2,long &x,long &y);
    //  The function will return YES if the point x,y is inside the polygon, or
    //  NO if it is not.  If the point is exactly on the edge of the polygon,
    //  then the function may return YES or NO.
    bool IsPointInPolygon(vector<MyPoint> poly,MyPoint pt);
    //鑻ョ偣a澶т簬鐐筨,鍗崇偣a鍦ㄧ偣b椤烘椂閽堟柟鍚�杩斿洖true,鍚﹀垯杩斿洖false
    bool PointCmp(const MyPoint &a,const MyPoint &b,const MyPoint &center);
    void ClockwiseSortPoints(vector<MyPoint> &vPoints);
    void Graham_scan(vector<MyPoint> PointSet,vector<MyPoint>& ch,int n,int &len);
    bool issimple();
    void checkconvex(vector<MyPoint>& polygon,vector<bool>& bc);
    bool isconvex();
    double area_of_polygon(vector<MyPoint> polygon);
    int intAreaCalc(vector<MyPoint> &vecPoly);

    //vector<Line> lineList;
    vector<MyPoint> pointList;
};

#endif // POLYGON_H
