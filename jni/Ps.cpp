
#include "Ps.h"


Ps::Ps()
{
}


Ps::~Ps()
{
}


void Ps::boostImage(Mat& src, Mat& dst, Point center)
{
	int width = src.cols;
	int heigh = src.rows;
	cvtColor(src, src, CV_RGBA2RGB);
	src.copyTo(dst);

	///cout << src.size() << "  " << img1.size() << "  " << img2.size() << endl;


	//【1】放大  
	int R1 = sqrtf(width*width + heigh*heigh) / 5; //直接关系到放大的力度,与R1成正比;  

	for (int y = 0; y < heigh; y++)
	{
		uchar *img1_p = dst.ptr<uchar>(y);
		for (int x = 0; x < width; x++)
		{
			int dis = norm(Point(x, y) - center);
			if (dis < R1)
			{
				int newX = (x - center.x)*dis / R1 + center.x;
				int newY = (y - center.y)*dis / R1 + center.y;

				img1_p[3 * x] = src.at<uchar>(newY, newX * 3);
				img1_p[3 * x + 1] = src.at<uchar>(newY, newX * 3 + 1);
				img1_p[3 * x + 2] = src.at<uchar>(newY, newX * 3 + 2);
			}
		}
	}
	cvtColor(dst, dst, CV_RGB2RGBA);
}

void Ps::shrinkImage(Mat& src, Mat& dst, Point center)
{
	int width = src.cols;
	int heigh = src.rows;
	
	cvtColor(src, src, CV_RGBA2RGB);
	src.copyTo(dst);
	///cout << src.size() << "  " << img1.size() << "  " << img2.size() << endl;

	for (int y = 0; y < heigh; y++)
	{
		uchar *img2_p = dst.ptr<uchar>(y);
		for (int x = 0; x < width; x++)
		{
			double theta = atan2((double)(y - center.y), (double)(x - center.x));//使用atan出现问题~  


			int R2 = sqrtf(norm(Point(x, y) - center)) * 12; //直接关系到挤压的力度，与R2成反比;  

			int newX = center.x + (int)(R2*cos(theta));

			int newY = center.y + (int)(R2*sin(theta));

			if (newX < 0) newX = 0;
			else if (newX >= width) newX = width - 1;
			if (newY < 0) newY = 0;
			else if (newY >= heigh) newY = heigh - 1;


			img2_p[3 * x] = src.at<uchar>(newY, newX * 3);
			img2_p[3 * x + 1] = src.at<uchar>(newY, newX * 3 + 1);
			img2_p[3 * x + 2] = src.at<uchar>(newY, newX * 3 + 2);
		}
	}
	cvtColor(dst, dst, CV_RGB2RGBA);
}

void Ps::radialZoomImage(Mat& src, Mat& dst, Point center)
{
	cvtColor(src, src, CV_RGBA2RGB);
	int num = 10;
	Mat src1u[3];
	split(src, src1u);

	int width = src.cols;
	int heigh = src.rows;
	src.copyTo(dst);



	for (int y = 0; y < heigh; y++)
	{

		uchar *imgP = dst.ptr<uchar>(y);

		for (int x = 0; x < width; x++)
		{
			int R = norm(Point(x, y) - center);
			double angle = atan2((double)(y - center.y), (double)(x - center.x));
			/*double len = sqrt((double)((y - center.y)*(y - center.y) + (x - center.x)*(x - center.x)));
			double cosa = 0.0f;
			double sina = 0.0f;
			if (abs(len) > 0.001f)
			{
			cosa = ((double)(x - center.x)) / len;
			sina = ((double)(y - center.y)) / len;
			}*/
			int tmp0 = 0, tmp1 = 0, tmp2 = 0;

			for (int i = 0; i<num; i++)      //num：均值力度 ，i为变化幅度;  
			{
				int tmpR = (R - i)>0 ? (R - i) : 0;

				int newX = tmpR*cos(angle) + center.x;
				int newY = tmpR*sin(angle) + center.y;

				if (newX<0)newX = 0;
				if (newX>width - 1)newX = width - 1;
				if (newY<0)newY = 0;
				if (newY>heigh - 1)newY = heigh - 1;

				tmp0 += src1u[0].at<uchar>(newY, newX);
				tmp1 += src1u[1].at<uchar>(newY, newX);
				tmp2 += src1u[2].at<uchar>(newY, newX);

			}
			imgP[3 * x] = (uchar)(tmp0 / num);
			imgP[3 * x + 1] = (uchar)(tmp1 / num);
			imgP[3 * x + 2] = (uchar)(tmp2 / num);
		}

	}
	cvtColor(dst, dst, CV_RGB2RGBA);
}

void Ps::rotateImage(Mat& src, Mat& dst, Point center)
{
	int num = 10;
	cvtColor(src, src, CV_RGBA2RGB);
	Mat src1u[3];
	split(src, src1u);
	src.copyTo(dst);

	int width = src.cols;
	int heigh = src.rows;



	for (int y = 0; y < heigh; y++)
	{

		uchar *imgP = dst.ptr<uchar>(y);

		for (int x = 0; x < width; x++)
		{
			int R = norm(Point(x, y) - center);
			double angle = atan2((double)(y - center.y), (double)(x - center.x));

			int tmp0 = 0, tmp1 = 0, tmp2 = 0;

			for (int i = 0; i < num; i++)  //均值力度;  
			{

				angle += 0.01;        //0.01控制变化频率，步长  

				int newX = R*cos(angle) + center.x;
				int newY = R*sin(angle) + center.y;

				if (newX<0)newX = 0;
				if (newX>width - 1)newX = width - 1;
				if (newY<0)newY = 0;
				if (newY>heigh - 1)newY = heigh - 1;

				tmp0 += src1u[0].at<uchar>(newY, newX);
				tmp1 += src1u[1].at<uchar>(newY, newX);
				tmp2 += src1u[2].at<uchar>(newY, newX);

			}
			imgP[3 * x] = (uchar)(tmp0 / num);
			imgP[3 * x + 1] = (uchar)(tmp1 / num);
			imgP[3 * x + 2] = (uchar)(tmp2 / num);
		}

	}
	cvtColor(dst, dst, CV_RGB2RGBA);
}
