/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * author: 	mnorst@foxmail.com
 * created:	2014/10/27
 * purpose:	旋转的纹理立方体
 */

// OpenGL ES 1.x code

#include <jni.h>
#include <android/log.h>

#include <GLES/gl.h>
#include <GLES/glext.h>

//#include <GLES2/gl2.h>
//#include <GLES2/gl2ext.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

using namespace std;


////////////////////////////////////////////////////////////////////
// File includes:
#include "ARDrawingContext.hpp"
#include "ARPipeline.hpp"
#include "DebugHelpers.hpp"

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <opencv2/opencv.hpp>

#include "Timer.h"
#include "Model/mesh.h"
#include "Model/pmesh.h"
#include "Globals.h"

extern ARPipeline pipeline;
extern ARDrawingContext drawingCtx;
extern CameraCalibration calibration;


extern Mesh *g_pMesh;
extern PMesh *g_pProgMesh;

/************************************************************************/
/*                             定义                                     */
/************************************************************************/

#define  LOG_TAG    "libgljni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

//初始化纹理数组


// 定义π
const GLfloat PI = 3.1415f;

// 定义顶点坐标
#define pos 0.2f

// 定义顶点坐标
// 一个正方体有8个顶点,6个面
#define one 1.0f
static GLfloat gVertices[] = {
		one, one, -one,
		-one, one, -one,
		one, one, one,

		-one, one, one,
		one, -one,one,
		-one, -one, one,

		one, -one, -one,
		-one, -one, -one,
		one, one,one,

		-one, one, one,
		one, -one, one,
		-one, -one, one,

		one, -one,-one,
		-one, -one, -one,
		one, one, -one,

		-one, one, -one,
		-one, one,one,
		-one, one, -one,

		-one, -one, one,
		-one, -one, -one,
		one, one,-one,

		one, one, one,
		one, -one, -one,
		one, -one, one
};

// 定义纹理坐标
// 纹理坐标原点会因不同系统环境而有所不同。
// 比如在iOS以及Android上，纹理坐标原点（0, 0）是在左上角
// 而在OS X上，纹理坐标的原点是在左下角
static GLfloat gTexCoords[] = {   
		0, one,
		one, one,
		0, 0,
		one, 0,

		0, one,
		one, one,
		0, 0,
		one, 0,

		0, one,
		one, one,
		0, 0,
		one, 0,

		0, one,
		one, one,
		0, 0,
		one, 0,

		0, one,
		one, one,
		0, 0,
		one, 0,

		0, one,
		one, one,
		0, 0,
		one, 0,


}; 

// 旋转角度
static GLfloat gAngle = 0.0f;
/************************************************************************/
/*                             C++代码                                  */
/************************************************************************/

static void printGLString(const char *name, GLenum s) {
	const char *v = (const char *) glGetString(s);
	LOGI("GL %s = %s\n", name, v);
}

static void checkGlError(const char* op) {
	for (GLint error = glGetError(); error; error = glGetError()) {
		LOGI("after %s() glError (0x%x)\n", op, error);
	}
}

bool init() {
	printGLString("Version", GL_VERSION);
	printGLString("Vendor", GL_VENDOR);
	printGLString("Renderer", GL_RENDERER);
	printGLString("Extensions", GL_EXTENSIONS);

	// 启用阴影平滑
	glShadeModel(GL_SMOOTH);

	// 黑色背景	
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);	

	// 设置深度缓存	
	glClearDepthf(1.0f);

	// 启用深度测试
	glEnable(GL_DEPTH_TEST);	

	// 所作深度测试的类型	
	glDepthFunc(GL_LEQUAL);	

	// 对透视进行修正	
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	

	glEnable(GL_LIGHT0); // default value is (1.0, 1.0, 1.0, 1.0)
	glEnable(GL_LIGHTING);

	return true;
}

static void _gluPerspective(GLfloat fovy, GLfloat aspect, GLfloat zNear, GLfloat zFar)
{
	GLfloat top = zNear * ((GLfloat) tan(fovy * PI / 360.0));
	GLfloat bottom = -top;
	GLfloat left = bottom * aspect;
	GLfloat right = top * aspect;
	glFrustumf(left, right, bottom, top, zNear, zFar);
}

void resize(int width, int height)
{
	// 防止被零除
	if (height==0)								
	{
		height=1;
	}

	int xstart=(width-screenWidth)/2;
	int ystart=(height-screenHeight)/2;
	// 重置当前的视口
	//glViewport(0, 0, width, height);
	//glViewport(xstart, ystart, 640, 480);


	//scale window to fit parent window
	double scalew=(double)width/(double)screenWidth;
	double scaleh=(double)height/(double)screenHeight;
	int dispw=width;
	int disph=height;
	if(scalew>scaleh)
	{
		dispw=(int)(scaleh*(double)screenWidth);
	}
	else
	{
		disph=(int)(scalew*(double)screenHeight);
	}
	int startx=(width-dispw)/2;
	int starty=(height-disph)/2;
	glViewport(startx, starty, dispw, disph);


	// 选择投影矩阵	
	glMatrixMode(GL_PROJECTION);	
	// 重置投影矩阵	
	glLoadIdentity();							

	// 设置视口的大小
	//_gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,100.0f);
	_gluPerspective(45.0f,(GLfloat)screenWidth/(GLfloat)screenHeight,0.1f,100.0f);

	// 选择模型观察矩阵
	glMatrixMode(GL_MODELVIEW);	

	// 重置模型观察矩阵
	glLoadIdentity();							
}

void renderFrame() {
	// 清除屏幕和深度缓存
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// 重置当前的模型观察矩阵
	glLoadIdentity();

	glTranslatef(0,0,-10.0f);
	glRotatef(gAngle, 0, 1.0F, 0);
	glRotatef(gAngle, 0, 0, 1.0F);
	glScalef(2.2f,2.2f,2.2f);

	// 启用顶点数组
	glEnableClientState(GL_VERTEX_ARRAY);
	//glEnableClientState(GL_COLOR_ARRAY);
	//glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	// 启用纹理映射
	//glEnable(GL_TEXTURE_2D);

	// 选择纹理
	//glBindTexture(GL_TEXTURE_2D, gTexture[0]);

	// 绘制正方体的六个面
	glVertexPointer(3,GL_FLOAT,0,gVertices);
	glTexCoordPointer(2, GL_FLOAT, 0, gTexCoords);

	for (int i = 0; i < 6; i++) {
		glDrawArrays(GL_TRIANGLE_STRIP, i * 4, 4);
	} 

	// 关闭顶点数组
	glDisableClientState(GL_VERTEX_ARRAY);
	//glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	//glDisableClientState(GL_OLOR_ARRAY);

	gAngle += 5.f;
}


bool processFrame1(cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx)
{
	// Clone image used for background (we will draw overlay on it)
	LOGE("begin clone");
	cv::Mat img = cameraFrame.clone();

	// Draw information:

	cv::putText(img, "RANSAC threshold: " + ToString(pipeline.m_patternDetector.homographyReprojectionThreshold) + "( Use'-'/'+' to adjust)", cv::Point(10, 30), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0,200,0));

	// Set a new camera frame:
	LOGE("begin update background");
	drawingCtx.updateBackground(img);

	// Find a pattern and update it's detection status:
	drawingCtx.isPatternPresent = pipeline.processFrame(cameraFrame);

	// Update a pattern pose:
	drawingCtx.patternPose = pipeline.getPatternLocation();

	Matrix33& r = drawingCtx.patternPose.r();
	Vector3& v = drawingCtx.patternPose.t();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cout << r.mat[i][j] << "  ";
		}
		cout << endl;
	}
	for (int i = 0; i < 3; i++)
	{
		cout << v.data[i] << "  ";
	}
	cout << endl;

	// Request redraw of the window:
	//drawingCtx.updateWindow();

	// Read the keyboard input:
	//int keyCode = cv::waitKey(5);

	bool shouldQuit = false;


	return shouldQuit;
}







void processSingleImage(cv::Mat& patternImage, CameraCalibration& calibration, cv::Mat& image)
{
	cv::Size frameSize(image.cols, image.rows);
	pipeline.init(patternImage, calibration);
	drawingCtx.init("Markerless AR", frameSize, calibration);

	bool shouldQuit = false;
	LOGE("begin process frame");
	shouldQuit = processFrame1(image, pipeline, drawingCtx);


}

/************************************************************************/
/*                          JNI代码                                     */
/************************************************************************/

extern "C" {
JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_resize(JNIEnv * env, jobject obj,  jint width, jint height);
JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_step(JNIEnv * env, jobject obj);
JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_init(JNIEnv * env, jobject obj);
JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_setTexture(JNIEnv * env, jclass obj, jintArray tex);
JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_test(JNIEnv * env, jobject obj);
};

JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_resize(JNIEnv * env, jobject obj,  jint width, jint height)
{
	LOGE("surface resize start");
	resize(width, height);
	//resize(640, 480);
	LOGE("surface resize end");
}

JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_step(JNIEnv * env, jobject obj)
{
	LOGE("surface step start");
	//renderFrame();
	LOGE("begin draw camera frame");
	//drawingCtx.drawCameraFrame();
	LOGE("begin draw augmented scene");
	drawingCtx.draw();
	LOGE("end draw augmented scene");
	//drawingCtx.drawCubeModel();
	LOGE("surface step end");
}

PMesh* makeMesh(string path)
{
	PMesh::EdgeCost g_edgemethod = PMesh::QUADRICTRI;
	char* charPath=path.c_str();
	Mesh* mesh = new Mesh(charPath);
	vector<vertex>& vert = g_pMesh->_vlist;

	if (mesh) mesh->Normalize(0.2f);// center mesh around the origin & shrink to fit
	cout<<"after normal"<<endl;


	PMesh* pmesh = new PMesh(mesh, g_edgemethod );
	return pmesh;
}

JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_init(JNIEnv * env, jobject obj)
{
	LOGE("surface init start");
	init();
	LOGE("surface init end");


	//g_pProgMesh = makeMesh("/sdcard/models/cow.ply");

	for(int i=0;i<modelPathList.size();i++)
	{
		PMesh* pmesh=makeMesh(modelPathList[i]);
		pmeshList.push_back(pmesh);
	}

	/*PMesh::EdgeCost g_edgemethod = PMesh::QUADRICTRI;
	g_pMesh = new Mesh("/sdcard/models/cow.ply");
	vector<vertex>& vert = g_pMesh->_vlist;

	if (g_pMesh) g_pMesh->Normalize(0.2f);// center mesh around the origin & shrink to fit
	cout<<"after normal"<<endl;


	g_pProgMesh = new PMesh(g_pMesh, g_edgemethod );*/
}

JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_setTexture(JNIEnv * env, jclass obj, jintArray tex)
{
	LOGE("surface set texture start");
	gTexture = (GLuint *)env->GetIntArrayElements(tex,0);
	LOGE("surface set texture end");
}

static bool isRedirect1=false;
void redirectStdOut1()
{
	if(isRedirect1==true)
	{
		return;
	}
	freopen("/sdcard/arStdout1.txt","w",stdout);
	cout<<"this is ar std out"<<endl;
	isRedirect1=true;
}

JNIEXPORT void JNICALL Java_com_example_ar_gljni_GLJNILib_test(JNIEnv * env, jobject obj)
{
	redirectStdOut1();
	cv::Mat patternImage = cv::imread("/sdcard/PyramidPattern.jpg");
	cv::Mat testImage = cv::imread("/sdcard/PyramidPatternTest.bmp");
	cvtColor(patternImage,patternImage,CV_BGR2RGB);
	cvtColor(testImage,testImage,CV_BGR2RGB);
	if (!testImage.empty()&&!patternImage.empty())
	{
		LOGE("begin process");
		processSingleImage(patternImage, calibration, testImage);
		LOGE("end process");
	}

}
