#ifndef LINE_H
#define LINE_H
#include "mypoint.h"

class MyLine
{
public:
    MyLine();
    ~MyLine();
    bool intersect(MyLine a,MyLine b);
    MyPoint s;
    MyPoint e;
};

#endif // LINE_H
