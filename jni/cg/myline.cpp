#include "myline.h"
#include <algorithm>
using namespace std;

MyLine::MyLine()
{

}

MyLine::~MyLine()
{

}

// 如果线段u和v相交(包括相交在端点处)时，返回true
//
//判断P1P2跨立Q1Q2的依据是：( P1 - Q1 ) × ( Q2 - Q1 ) * ( Q2 - Q1 ) × ( P2 - Q1 ) >= 0。
//判断Q1Q2跨立P1P2的依据是：( Q1 - P1 ) × ( P2 - P1 ) * ( P2 - P1 ) × ( Q2 - P1 ) >= 0。
bool MyLine::intersect(MyLine u,MyLine v)
{
    return( (max(u.s.x,u.e.x)>=min(v.s.x,v.e.x))&&                     //排斥实验
            (max(v.s.x,v.e.x)>=min(u.s.x,u.e.x))&&
            (max(u.s.y,u.e.y)>=min(v.s.y,v.e.y))&&
            (max(v.s.y,v.e.y)>=min(u.s.y,u.e.y))&&
            (v.s.multiply(v.s,u.e,u.s)*v.s.multiply(u.e,v.e,u.s)>=0)&&         //跨立实验
            (v.s.multiply(u.s,v.e,v.s)*v.s.multiply(v.e,u.e,v.s)>=0));
}
