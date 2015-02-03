#include "mypolygon.h"
#include <algorithm>
#include <iostream>
using namespace std;

MyPolygon::MyPolygon()
{
    
}

MyPolygon::MyPolygon(vector<MyPoint> pList)
{
    pointList=pList;
}

MyPolygon::~MyPolygon()
{
    
}

bool MyPolygon::intersect(MyPolygon& a,MyPolygon& b,MyPolygon& interPoly)
{
    bool success=intersect(a.pointList,b.pointList,interPoly.pointList);

    return success;
}

bool MyPolygon::intersect(vector<MyPoint>& poly1,vector<MyPoint>& poly2,vector<MyPoint>& interPoly)
{
    if (poly1.size() < 3 || poly2.size() < 3)
    {
        return false;
    }

    long x,y;
    //璁＄畻澶氳竟褰氦鐐�
    for (int i = 0;i < poly1.size();i++)
    {
        int poly1_next_idx = (i + 1) % poly1.size();
        for (int j = 0;j < poly2.size();j++)
        {
            int poly2_next_idx = (j + 1) % poly2.size();
            if (GetCrossPoint(poly1[i],poly1[poly1_next_idx],
                              poly2[j],poly2[poly2_next_idx],
                              x,y))
            {
                interPoly.push_back(MyPoint(x,y));
            }
        }
    }

    //璁＄畻澶氳竟褰㈠唴閮ㄧ偣
    for(int i = 0;i < poly1.size();i++)
    {
        if (IsPointInPolygon(poly2,poly1[i]))
        {
            interPoly.push_back(poly1[i]);
        }
    }
    for (int i = 0;i < poly2.size();i++)
    {
        if (IsPointInPolygon(poly1,poly2[i]))
        {
            interPoly.push_back(poly2[i]);
        }
    }

    if(interPoly.size() <= 0)
        return false;

    //鐐归泦鎺掑簭
    //ClockwiseSortPoints(interPoly);
    int orisize,aftersize;
    vector<MyPoint> afterPoint;
    Graham_scan(interPoly,afterPoint,interPoly.size(),aftersize);
    interPoly=afterPoint;
    return true;
}





//鎺掓枼瀹為獙
bool MyPolygon::IsRectCross(const MyPoint &p1,const MyPoint &p2,const MyPoint &q1,const MyPoint &q2)
{
    bool ret =  min(p1.x,p2.x) <= max(q1.x,q2.x)    &&
                min(q1.x,q2.x) <= max(p1.x,p2.x) &&
                min(p1.y,p2.y) <= max(q1.y,q2.y) &&
                min(q1.y,q2.y) <= max(p1.y,p2.y);
    return ret;
}
//璺ㄧ珛鍒ゆ柇
bool MyPolygon::IsLineSegmentCross(const MyPoint &pFirst1,const MyPoint &pFirst2,const MyPoint &pSecond1,const MyPoint &pSecond2)
{
    long line1,line2;
    line1 = pFirst1.x * (pSecond1.y - pFirst2.y) +
        pFirst2.x * (pFirst1.y - pSecond1.y) +
        pSecond1.x * (pFirst2.y - pFirst1.y);
    line2 = pFirst1.x * (pSecond2.y - pFirst2.y) +
        pFirst2.x * (pFirst1.y - pSecond2.y) +
        pSecond2.x * (pFirst2.y - pFirst1.y);
    if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
        return false;

    line1 = pSecond1.x * (pFirst1.y - pSecond2.y) +
        pSecond2.x * (pSecond1.y - pFirst1.y) +
        pFirst1.x * (pSecond2.y - pSecond1.y);
    line2 = pSecond1.x * (pFirst2.y - pSecond2.y) +
        pSecond2.x * (pSecond1.y - pFirst2.y) +
        pFirst2.x * (pSecond2.y - pSecond1.y);
    if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
        return false;
    return true;
}

bool MyPolygon::GetCrossPoint(const MyPoint &p1,const MyPoint &p2,const MyPoint &q1,const MyPoint &q2,long &x,long &y)
{
    if(IsRectCross(p1,p2,q1,q2))
    {
        if (IsLineSegmentCross(p1,p2,q1,q2))
        {
            //姹備氦鐐�
            long tmpLeft,tmpRight;
            tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
            tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);

            x = (int)((double)tmpRight/(double)tmpLeft);

            tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
            tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x- p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y);
            y = (int)((double)tmpRight/(double)tmpLeft);
            return true;
        }
    }
    return false;
}

//  The function will return YES if the point x,y is inside the polygon, or
//  NO if it is not.  If the point is exactly on the edge of the polygon,
//  then the function may return YES or NO.
bool MyPolygon::IsPointInPolygon(vector<MyPoint> poly,MyPoint pt)
{
    int i,j;
    bool c = false;
    for (i = 0,j = poly.size() - 1;i < poly.size();j = i++)
    {
        if ((((poly[i].y <= pt.y) && (pt.y < poly[j].y)) ||
            ((poly[j].y <= pt.y) && (pt.y < poly[i].y)))
            && (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y)/(poly[j].y - poly[i].y) + poly[i].x))
        {
            c = !c;
        }
    }
    return c;
}


//鑻ョ偣a澶т簬鐐筨,鍗崇偣a鍦ㄧ偣b椤烘椂閽堟柟鍚�杩斿洖true,鍚﹀垯杩斿洖false
bool MyPolygon::PointCmp(const MyPoint &a,const MyPoint &b,const MyPoint &center)
{
    if (a.x >= 0 && b.x < 0)
        return true;
    if (a.x == 0 && b.x == 0)
        return a.y > b.y;
    //鍚戦噺OA鍜屽悜閲廜B鐨勫弶绉�
    int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
    if (det < 0)
        return true;
    if (det > 0)
        return false;
    //鍚戦噺OA鍜屽悜閲廜B鍏辩嚎锛屼互璺濈鍒ゆ柇澶у皬
    int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
    int d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
    return d1 > d2;
}
void MyPolygon::ClockwiseSortPoints(vector<MyPoint> &vPoints)
{
    //璁＄畻閲嶅績
    MyPoint center;
    double x = 0,y = 0;
    for (int i = 0;i < vPoints.size();i++)
    {
        x += vPoints[i].x;
        y += vPoints[i].y;
    }
    center.x = (int)x/vPoints.size();
    center.y = (int)y/vPoints.size();

    //鍐掓场鎺掑簭
    for(int i = 0;i < vPoints.size() - 1;i++)
    {
        for (int j = 0;j < vPoints.size() - i - 1;j++)
        {
            if (PointCmp(vPoints[j],vPoints[j+1],center))
            {
                MyPoint tmp = vPoints[j];
                vPoints[j] = vPoints[j + 1];
                vPoints[j + 1] = tmp;
            }
        }
    }
}

/*double dist(Point p1,Point p2)                // 杩斿洖涓ょ偣涔嬮棿娆ф皬璺濈
{
    return( sqrt( (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y) ) );
}*/

/******************************************************************************
r=multiply(sp,ep,op),寰楀埌(sp-op)鍜�ep-op)鐨勫弶绉�
r>0锛歟p鍦ㄧ煝閲弌psp鐨勯�鏃堕拡鏂瑰悜锛�
r=0锛歰pspep涓夌偣鍏辩嚎锛�
r<0锛歟p鍦ㄧ煝閲弌psp鐨勯『鏃堕拡鏂瑰悜
*******************************************************************************/
/*double multiply(Point sp,Point ep,Point op)
{
    return((sp.x-op.x)*(ep.y-op.y)-(ep.x-op.x)*(sp.y-op.y));
}*/

/**********************************************
瀵绘壘鍑稿寘鐨刧raham 鎵弿娉�
PointSet涓鸿緭鍏ョ殑鐐归泦锛�
ch涓鸿緭鍑虹殑鍑稿寘涓婄殑鐐归泦锛屾寜鐓ч�鏃堕拡鏂瑰悜鎺掑垪;
n涓篜ointSet涓殑鐐圭殑鏁扮洰
len涓鸿緭鍑虹殑鍑稿寘涓婄殑鐐圭殑涓暟
**********************************************/
void MyPolygon::Graham_scan(vector<MyPoint> PointSet,vector<MyPoint>& ch,int n,int &len)
{
    int i,j,k=0,top=2;
    ch.resize(PointSet.size());
    MyPoint tmp;
    // 閫夊彇PointSet涓瓂鍧愭爣鏈�皬鐨勭偣PointSet[k]锛屽鏋滆繖鏍风殑鐐规湁澶氫釜锛屽垯鍙栨渶宸﹁竟鐨勪竴涓�
    for(i=1;i<n;i++)
        if ( PointSet[i].y<PointSet[k].y || (PointSet[i].y==PointSet[k].y) && (PointSet[i].x<PointSet[k].x) )
            k=i;
    tmp=PointSet[0];
    PointSet[0]=PointSet[k];
    PointSet[k]=tmp; // 鐜板湪PointSet涓瓂鍧愭爣鏈�皬鐨勭偣鍦≒ointSet[0]
    for (i=1;i<n-1;i++) /* 瀵归《鐐规寜鐓х浉瀵筆ointSet[0]鐨勬瀬瑙掍粠灏忓埌澶ц繘琛屾帓搴忥紝鏋佽鐩稿悓鐨勬寜鐓ц窛绂籔ointSet[0]浠庤繎鍒拌繙杩涜鎺掑簭 */
    {
        k=i;
        for (j=i+1;j<n;j++)
            if ( PointSet[j].multiply(PointSet[j],PointSet[k],PointSet[0])>0 ||  // 鏋佽鏇村皬
                (PointSet[j].multiply(PointSet[j],PointSet[k],PointSet[0])==0) && /* 鏋佽鐩哥瓑锛岃窛绂绘洿鐭�*/
                PointSet[j].dist(PointSet[0],PointSet[j])<PointSet[j].dist(PointSet[0],PointSet[k])
               )
                k=j;
        tmp=PointSet[i];
        PointSet[i]=PointSet[k];
        PointSet[k]=tmp;
    }
    ch[0]=PointSet[0];
    ch[1]=PointSet[1];
    ch[2]=PointSet[2];
    for (i=3;i<n;i++)
    {
        while (PointSet[i].multiply(PointSet[i],ch[top],ch[top-1])>=0)
            top--;
        ch[++top]=PointSet[i];
    }
    len=top+1;
    ch.resize(len);
}

// 濡傛灉绾挎u鍜寁鐩镐氦(鍖呮嫭鐩镐氦鍦ㄧ鐐瑰)鏃讹紝杩斿洖true
//
//鍒ゆ柇P1P2璺ㄧ珛Q1Q2鐨勪緷鎹槸锛� P1 - Q1 ) 脳 ( Q2 - Q1 ) * ( Q2 - Q1 ) 脳 ( P2 - Q1 ) >= 0銆�
//鍒ゆ柇Q1Q2璺ㄧ珛P1P2鐨勪緷鎹槸锛� Q1 - P1 ) 脳 ( P2 - P1 ) * ( P2 - P1 ) 脳 ( Q2 - P1 ) >= 0銆�
/*bool intersect(Line u,Line v)
{
    return( (max(u.s.x,u.e.x)>=min(v.s.x,v.e.x))&&                     //鎺掓枼瀹為獙
            (max(v.s.x,v.e.x)>=min(u.s.x,u.e.x))&&
            (max(u.s.y,u.e.y)>=min(v.s.y,v.e.y))&&
            (max(v.s.y,v.e.y)>=min(u.s.y,u.e.y))&&
            (v.s.multiply(v.s,u.e,u.s)*v.s.multiply(u.e,v.e,u.s)>=0)&&         //璺ㄧ珛瀹為獙
            (v.s.multiply(u.s,v.e,v.s)*v.s.multiply(v.e,u.e,v.s)>=0));
}*/

bool MyPolygon::issimple()
{
    int i,cn;
    MyLine l1,l2;
    int vcount=pointList.size();
    for(i=0;i<vcount;i++)
    {
        l1.s=pointList[i];
        l1.e=pointList[(i+1)%vcount];
        cn=vcount-3;
        while(cn)
        {
            l2.s=pointList[(i+2)%vcount];
            l2.e=pointList[(i+3)%vcount];
            if(l1.intersect(l1,l2))
                break;
            cn--;
        }
        if(cn)
            return false;
    }
    return true;
}

// 杩斿洖鍊硷細鎸夎緭鍏ラ『搴忚繑鍥炲杈瑰舰椤剁偣鐨勫嚫鍑规�鍒ゆ柇锛宐c[i]=1,iff:绗琲涓《鐐规槸鍑搁《鐐�
void MyPolygon::checkconvex(vector<MyPoint>& polygon,vector<bool>& bc)
{
    int i,index=0;
    bc.resize(polygon.size());
    MyPoint tp=polygon[0];
    int vcount=polygon.size();
    for(i=1;i<vcount;i++) // 瀵绘壘绗竴涓嚫椤剁偣
    {
        if(polygon[i].y<tp.y||(polygon[i].y == tp.y&&polygon[i].x<tp.x))
        {
            tp=polygon[i];
            index=i;
        }
    }
    int count=vcount-1;
    bc[index]=1;
    while(count) // 鍒ゆ柇鍑稿嚬鎬�
    {
        if(polygon[0].multiply(polygon[(index+1)%vcount],polygon[(index+2)%vcount],polygon[index])>=0 )
            bc[(index+1)%vcount]=1;
        else
            bc[(index+1)%vcount]=0;
        index++;
        count--;
    }
}

/// 返回多边形面积(signed)；输入顶点按逆时针排列时，返回正值；否则返回负值
double MyPolygon::area_of_polygon1(vector<MyPoint> polygon)
{
    //cout<<"begin calc area"<<endl;
    int vcount=polygon.size();
    int i;
    double s;
    if (vcount<3)
        return 0;
    s=polygon[0].y*(polygon[vcount-1].x-polygon[1].x);
    //cout<<s<<endl;
    for (i=1;i<vcount;i++)
    {
        double incre=polygon[i].y*(polygon[(i-1)].x-polygon[(i+1)%vcount].x);
        s+=incre;
        cout<<incre<<"  "<<s<<endl;
    }
    //cout<<"finally area1"<<endl;
    //cout<<abs(s/2)<<endl;
    return abs(s/2);
}

double area_of_triangle(MyPoint p1, MyPoint p2, MyPoint p3)
{
	double a = p1.dist(p1, p2);
	double b = p2.dist(p2, p3);
	double c = p1.dist(p1, p3);
	double p = (a + b + c) / 2;
	double s = sqrt(p*(p - a)*(p - b)*(p - c));
	return s;
}

// 返回多边形面积(signed)；输入顶点按逆时针排列时，返回正值；否则返回负值
double MyPolygon::area_of_polygon(vector<MyPoint> polygon)
{
	if (polygon.size() < 3)
	{
		return 0;
	}
	double s = 0;
	for (int i = 0; i < polygon.size() - 2; i++)
	{
		s += area_of_triangle(polygon[0], polygon[i+1], polygon[i+2]);
	}
	return s;
}

int MyPolygon::intAreaCalc(vector<MyPoint> &vecPoly)
{
    //cout<<"begin area"<<endl;
    int iCycle,iCount,iArea;
    iCycle=0;
    iArea=0;
    iCount=vecPoly.size();

    for(iCycle=0;iCycle<iCount;iCycle++)
    {
        iArea=iArea+(vecPoly[iCycle].x*vecPoly[(iCycle+1) % iCount].y-vecPoly[(iCycle+1) % iCount].x*vecPoly[iCycle].y);
        cout<<iArea<<endl;
    }
    //cout<<"finally area2"<<endl;
    //cout<<abs(0.5*iArea)<<endl;
    return abs(0.5*iArea);
}

// 杩斿洖鍊硷細澶氳竟褰olygon鏄嚫澶氳竟褰㈡椂锛岃繑鍥瀟rue
bool MyPolygon::isconvex()
{
    int orisize,aftersize;
    vector<MyPoint> afterPoint;
    Graham_scan(pointList,afterPoint,pointList.size(),aftersize);
    //cout<<"ori1"<<endl;
    //cout<<area_of_polygon(pointList)<<endl;
    //cout<<"after1"<<endl;
    //cout<<area_of_polygon(afterPoint)<<endl;
    //cout<<"ori2"<<endl;
    //cout<<intAreaCalc(pointList)<<endl;
    //cout<<"after2"<<endl;
    //cout<<intAreaCalc(afterPoint)<<endl;
    if(pointList.size()==afterPoint.size())
    {
        return true;
    }
    return false;
}
