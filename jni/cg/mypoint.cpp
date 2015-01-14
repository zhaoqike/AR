#include "mypoint.h"
#include <algorithm>

using namespace std;

MyPoint::MyPoint()
{

}

MyPoint::MyPoint(int xx, int yy)
{
    x=xx;
    y=yy;
}

MyPoint::~MyPoint()
{

}

double MyPoint::dist(MyPoint p1,MyPoint p2)                // 返回两点之间欧氏距离
{
    return( sqrt( (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y) ) );
}

/******************************************************************************
r=multiply(sp,ep,op),得到(sp-op)和(ep-op)的叉积
r>0：ep在矢量opsp的逆时针方向；
r=0：opspep三点共线；
r<0：ep在矢量opsp的顺时针方向
*******************************************************************************/
double MyPoint::multiply(MyPoint sp,MyPoint ep,MyPoint op)
{
    return((sp.x-op.x)*(ep.y-op.y)-(ep.x-op.x)*(sp.y-op.y));
}
