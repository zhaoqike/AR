#ifndef POINT_H
#define POINT_H


class MyPoint
{
public:
    MyPoint();
    MyPoint(int xx,int yy);
    ~MyPoint();
    double dist(MyPoint p1,MyPoint p2);                // 返回两点之间欧氏距离
    double multiply(MyPoint sp,MyPoint ep,MyPoint op);

    int x;
    int y;
};

#endif // POINT_H
