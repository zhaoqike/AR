
////////////////////////////////////////////////////////////////////
// File includes:

#include "ARDrawingContext.hpp"

#include <iostream>
using namespace std;

////////////////////////////////////////////////////////////////////
// Standard includes:
//#include <Windows.h>
//#include <gl/h>
//#include <gl/glu.h>

#include "Model/pmesh.h"
#include "ARPipeline.hpp"

#ifndef WIN32
#include <jni.h>
#include <android/log.h>
#include <GLES/gl.h>
#include <GLES/glext.h>
#define  LOG_TAG    "libgljni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#else
#include <Windows.h>
#include <gl/gl.h>
#include <gl/glu.h>
#endif

//#ifndef WIN32
extern GLuint *gTexture;
extern PMesh *g_pProgMesh;
extern ARPipeline pipeline;
//#endif

void ARDrawingContextDrawCallback(void* param) {
	ARDrawingContext * ctx = static_cast<ARDrawingContext*>(param);
	if (ctx) {
		ctx->draw();
	}
}

ARDrawingContext::ARDrawingContext() {

}

ARDrawingContext::ARDrawingContext(string windowName, Size frameSize,
	const CameraCalibration& c) :
	m_isTextureInitialized(false), m_calibration(c), m_windowName(
	windowName) {
	// Create window with OpenGL support
	namedWindow(windowName, WINDOW_OPENGL);

	// Resize it exactly to video size
	resizeWindow(windowName, frameSize.width, frameSize.height);

	// Initialize OpenGL draw callback:
	setOpenGlContext(windowName);
	setOpenGlDrawCallback(windowName, ARDrawingContextDrawCallback, this);
}

void ARDrawingContext::init(string windowName, Size frameSize,
	const CameraCalibration& c) {
	m_isTextureInitialized = false;
	m_calibration = c;
	m_windowName = windowName;
#ifdef WIN32
	// Create window with OpenGL support
	namedWindow(windowName, WINDOW_OPENGL);

	// Resize it exactly to video size
	resizeWindow(windowName, frameSize.width, frameSize.height);

	// Initialize OpenGL draw callback:
	setOpenGlContext(windowName);
	setOpenGlDrawCallback(windowName, ARDrawingContextDrawCallback, this);
#endif
}

ARDrawingContext::~ARDrawingContext() {
	setOpenGlDrawCallback(m_windowName, 0, 0);
}

void ARDrawingContext::updateBackground(const Mat& frame) {
	frame.copyTo(m_backgroundImage);
	width = frame.cols;
	height = frame.rows;
	cout << "update background" << endl;

}

void ARDrawingContext::updateWindow() {
	cv::updateWindow(m_windowName);
}

void ARDrawingContext::draw() {
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // Clear entire screen:
	drawCameraFrame();                                  // Render background

	/*for (int i = 0; i < pipeline.m_patternDetector.m_pattern.keyframeList.size(); i++) {
		Point3f & center = pipeline.m_patternDetector.m_pattern.keyframeList[i].center;
		drawAugmentedScene(center.x, center.y, center.z);
	}*/

	vector<int>& matchedKfs = pipeline.m_patternDetector.nowMatchedKeyframes;
	for (int i = 0; i < matchedKfs.size();i++)
	{
		int index = matchedKfs[i];
		Point3f & center = pipeline.m_patternDetector.m_pattern.keyframeList[index].center;
		drawAugmentedScene(center.x, center.y, center.z);
	}
	cout << "begin cout center data" << endl;
	for (int i = 0; i < pipeline.m_patternDetector.m_pattern.keyframeList.size(); i++) {
		Point3f & center = pipeline.m_patternDetector.m_pattern.keyframeList[i].center;
		cout << "center " << i << " : " << center.x << "  " << center.y << "  " << center.z << endl;
	}
	cout << "end cout center data" << endl;
	//drawAugmentedScene();                               // Draw AR
	glFlush();
	cout << "end draw" << endl;
}

void ARDrawingContext::drawCameraFrame() {
	if (m_backgroundImage.cols == 0 || m_backgroundImage.rows == 0) {
		cout << "background image is null" << endl;
		return;
	}
	// Initialize texture for background image
	if (!m_isTextureInitialized) {
		glGenTextures(1, &m_backgroundTextureId);
		glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		m_isTextureInitialized = true;
		cout << "init draw" << endl;
	}

	int w = m_backgroundImage.cols;
	int h = m_backgroundImage.rows;
	cout << "width: " << w << " height: " << h << " channel: " << m_backgroundImage.channels() << endl;
	cvtColor(m_backgroundImage, m_backgroundImage, CV_BGR2RGB);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);

	// Upload new texture data:
	if (m_backgroundImage.channels() == 3)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, m_backgroundImage.data);
	else if (m_backgroundImage.channels() == 4)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_backgroundImage.data);
	else if (m_backgroundImage.channels() == 1)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, m_backgroundImage.data);

	const GLfloat bgTextureVertices[] = { 0, 0, w, 0, 0, h, w, h };
	const GLfloat bgTextureCoords[] = { 1, 0, 1, 1, 0, 0, 0, 1 };
	const GLfloat proj[] = { 0, -2.f / w, 0, 0, -2.f / h, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(proj);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);

	// Update attribute values.
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisable(GL_DEPTH_TEST);

	glVertexPointer(2, GL_FLOAT, 0, bgTextureVertices);
	glTexCoordPointer(2, GL_FLOAT, 0, bgTextureCoords);

	glColor4f(1, 1, 1, 1);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
}

/*void ARDrawingContext::drawAugmentedScene()
 {
 // Init augmentation projection
 Matrix44 projectionMatrix;
 int w = width;
 int h = height;
 buildProjectionMatrix(m_calibration, w, h, projectionMatrix);

 glMatrixMode(GL_PROJECTION);
 glLoadMatrixf(projectionMatrix.data);
 cout<<"projectionMatrix: "<<endl;
 for(int i=0;i<16;i++){
 cout<<projectionMatrix.data[i]<<"  ";
 }
 cout<<endl;

 glMatrixMode(GL_MODELVIEW);
 glLoadIdentity();
 cout<<"is pattern present: "<<isPatternPresent<<endl;
 if (isPatternPresent)
 {
 cout<<"pattern is present"<<endl;
 // Set the pattern transformation
 Matrix44 glMatrix = patternPose.getMat44();
 cout<<"glMatrix: "<<endl;
 for(int i=0;i<16;i++)
 {
 cout<<glMatrix.data[i]<<"  ";
 }
 cout<<endl;
 glLoadMatrixf(reinterpret_cast<const GLfloat*>(&glMatrix.data[0]));
 //glScalef(0.2f, 0.2f, 0.2f);
 // Render model
 drawCoordinateAxis();
 //drawCubeModel();
 drawMesh();
 }
 }*/

void ARDrawingContext::drawAugmentedScene(float x, float y, float z) {
	// Init augmentation projection
	Matrix44 projectionMatrix;
	int w = width;
	int h = height;
	buildProjectionMatrix(m_calibration, w, h, projectionMatrix);

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(projectionMatrix.data);
	cout << "projectionMatrix: " << endl;
	for (int i = 0; i < 16; i++) {
		cout << projectionMatrix.data[i] << "  ";
	}
	cout << endl;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	cout << "is pattern present: " << isPatternPresent << endl;
	if (isPatternPresent) {
		cout << "pattern is present" << endl;
		// Set the pattern transformation
		Matrix44 glMatrix = patternPose.getMat44();
		cout << "glMatrix: " << endl;
		for (int i = 0; i < 16; i++) {
			cout << glMatrix.data[i] << "  ";
		}
		cout << endl;
		glLoadMatrixf(reinterpret_cast<const GLfloat*>(&glMatrix.data[0]));
		glTranslatef(-x, -y, z);
#ifndef WIN32
		//glScalef(0.2f, 0.2f, 0.2f);
#endif
		// Render model
		//drawCoordinateAxis();
#ifdef WIN32
		drawCubeModel();
#else
		drawMesh();
#endif
	}
}

void ARDrawingContext::buildProjectionMatrix(const CameraCalibration& calibration, int screen_width, int screen_height, Matrix44& projectionMatrix) {
	float nearPlane = 0.01f;  // Near clipping distance
	float farPlane = 100.0f;  // Far clipping distance

	// Camera parameters
	float f_x = calibration.fx(); // Focal length in x axis
	float f_y = calibration.fy(); // Focal length in y axis (usually the same?)
	float c_x = calibration.cx(); // Camera primary point x
	float c_y = calibration.cy(); // Camera primary point y

	projectionMatrix.data[0] = -2.0f * f_x / screen_width;
	projectionMatrix.data[1] = 0.0f;
	projectionMatrix.data[2] = 0.0f;
	projectionMatrix.data[3] = 0.0f;

	projectionMatrix.data[4] = 0.0f;
	projectionMatrix.data[5] = 2.0f * f_y / screen_height;
	projectionMatrix.data[6] = 0.0f;
	projectionMatrix.data[7] = 0.0f;

	projectionMatrix.data[8] = 2.0f * c_x / screen_width - 1.0f;
	projectionMatrix.data[9] = 2.0f * c_y / screen_height - 1.0f;
	projectionMatrix.data[10] = -(farPlane + nearPlane) / (farPlane - nearPlane);
	projectionMatrix.data[11] = -1.0f;

	projectionMatrix.data[12] = 0.0f;
	projectionMatrix.data[13] = 0.0f;
	projectionMatrix.data[14] = -2.0f * farPlane * nearPlane / (farPlane - nearPlane);
	projectionMatrix.data[15] = 0.0f;
}

void ARDrawingContext::drawCoordinateAxis() {
#ifndef WIN32 //android
	float lineX[] = { 0, 0, 0, 1, 0, 0 };
	float lineY[] = { 0, 0, 0, 0, 1, 0 };
	float lineZ[] = { 0, 0, 0, 0, 0, 1 };

	int lineXcolor[] = { 65535, 0, 0, 0, 65535, 0, 0, 0 };
	int lineYcolor[] = { 0, 65535, 0, 0, 0, 65535, 0, 0 };
	int lineZcolor[] = { 0, 0, 65535, 0, 0, 0, 65535, 0 };

	glLineWidth(2);

	glEnableClientState (GL_VERTEX_ARRAY);
	glEnableClientState (GL_COLOR_ARRAY);

	glVertexPointer(3, GL_FLOAT, 0, lineX);
	glColorPointer(4, GL_FIXED, 0, lineXcolor);
	glDrawArrays(GL_LINES, 0, 2);
	//glDisableClientState(GL_VERTEX_ARRAY);

	//glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, lineY);
	glColorPointer(4, GL_FIXED, 0, lineYcolor);
	glDrawArrays(GL_LINES, 0, 2);
	//glDisableClientState(GL_VERTEX_ARRAY);

	//glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, lineZ);
	glColorPointer(4, GL_FIXED, 0, lineZcolor);
	glDrawArrays(GL_LINES, 0, 2);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
#else //windows
	static float lineX[] = { 0, 0, 0, 1, 0, 0 };
	static float lineY[] = { 0, 0, 0, 0, 1, 0 };
	static float lineZ[] = { 0, 0, 0, 0, 0, 1 };

	glLineWidth(2);

	glBegin(GL_LINES);

	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3fv(lineX);
	glVertex3fv(lineX + 3);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3fv(lineY);
	glVertex3fv(lineY + 3);

	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3fv(lineZ);
	glVertex3fv(lineZ + 3);

	glEnd();
#endif
}

void ARDrawingContext::drawCubeModel() {
#ifndef WIN32
	glScalef(0.25, 0.25, 0.25);
	//glTranslatef(0,0, 1);
	cout << "begin draw cube model" << endl;
	//初始化纹理数组

	// 定义π
	const GLfloat PI = 3.1415f;

	// 定义顶点坐标
#define pos 1.0f

	// 定义顶点坐标
	// 一个正方体有8个顶点,6个面
#define one 1.0f
	static GLfloat gVertices[] = {
		one, one, -one, -one, one, -one,
		one, one, one,

		-one, one, one,
		one, -one, one, -one, -one, one,

		one, -one, -one, -one, -one, -one,
		one, one, one,

		-one, one, one,
		one, -one, one, -one, -one, one,

		one, -one, -one, -one, -one, -one,
		one, one, -one,

		-one, one, -one, -one, one, one, -one, one, -one,

		-one, -one, one, -one, -one, -one,
		one, one, -one,

		one, one, one,
		one, -one, -one,
		one, -one, one };

	// 定义纹理坐标
	// 纹理坐标原点会因不同系统环境而有所不同。
	// 比如在iOS以及Android上，纹理坐标原点（0, 0）是在左上角
	// 而在OS X上，纹理坐标的原点是在左下角
	static GLfloat gTexCoords[] = { 0, one,
		one, one, 0, 0,
		one, 0,

		0, one,
		one, one, 0, 0,
		one, 0,

		0, one,
		one, one, 0, 0,
		one, 0,

		0, one,
		one, one, 0, 0,
		one, 0,

		0, one,
		one, one, 0, 0,
		one, 0,

		0, one,
		one, one, 0, 0,
		one, 0,

	};

	// 旋转角度
	//static GLfloat gAngle = 0.0f;

	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// 重置当前的模型观察矩阵
	//glLoadIdentity();

	//glTranslatef(0,0,-10.0f);
	//glRotatef(gAngle, 0, 1.0F, 0);
	//glRotatef(gAngle, 0, 0, 1.0F);
	//glScalef(2.2f,2.2f,2.2f);

	// 启用顶点数组
	glEnableClientState (GL_VERTEX_ARRAY);
	//glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState (GL_TEXTURE_COORD_ARRAY);

	// 启用纹理映射
	glEnable (GL_TEXTURE_2D);

	// 选择纹理
	cout << "begin bind texture" << endl;
	glBindTexture(GL_TEXTURE_2D, gTexture[0]);

	// 绘制正方体的六个面
	glVertexPointer(3, GL_FLOAT, 0, gVertices);
	glTexCoordPointer(2, GL_FLOAT, 0, gTexCoords);
	cout << "begin draw array" << endl;
	for (int i = 0; i < 6; i++) {
		glDrawArrays(GL_TRIANGLE_STRIP, i * 4, 4);
	}
	cout << "end draw array" << endl;

	// 关闭顶点数组
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	//glDisableClientState(GL_OLOR_ARRAY);

	//gAngle += 5.f;
	cout << "angle: " << endl;

#else
	static const GLfloat LightAmbient[] = { 0.25f, 0.25f, 0.25f, 1.0f }; // Ambient Light Values
	static const GLfloat LightDiffuse[] = { 0.1f, 0.1f, 0.1f, 1.0f }; // Diffuse Light Values
	static const GLfloat LightPosition[] = { 0.0f, 0.0f, 2.0f, 1.0f }; // Light Position

	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_POLYGON_BIT);

	glColor4f(0.2f, 0.35f, 0.3f, 0.75f);// Full Brightness, 50% Alpha ( NEW )
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);// Blending Function For Translucency Based On Source Alpha
	glEnable(GL_BLEND);

	glShadeModel(GL_SMOOTH);

	glEnable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
	glEnable(GL_COLOR_MATERIAL);

	glScalef(0.25, 0.25, 0.25);
	glTranslatef(0, 0, 1);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_QUADS);
	// Front Face
	glNormal3f(0.0f, 0.0f, 1.0f);// Normal Pointing Towards Viewer
	glVertex3f(-1.0f, -1.0f, 1.0f);// Point 1 (Front)
	glVertex3f(1.0f, -1.0f, 1.0f);// Point 2 (Front)
	glVertex3f(1.0f, 1.0f, 1.0f);// Point 3 (Front)
	glVertex3f(-1.0f, 1.0f, 1.0f);// Point 4 (Front)
	// Back Face
	glNormal3f(0.0f, 0.0f, -1.0f);// Normal Pointing Away From Viewer
	glVertex3f(-1.0f, -1.0f, -1.0f);// Point 1 (Back)
	glVertex3f(-1.0f, 1.0f, -1.0f);// Point 2 (Back)
	glVertex3f(1.0f, 1.0f, -1.0f);// Point 3 (Back)
	glVertex3f(1.0f, -1.0f, -1.0f);// Point 4 (Back)
	// Top Face
	glNormal3f(0.0f, 1.0f, 0.0f);// Normal Pointing Up
	glVertex3f(-1.0f, 1.0f, -1.0f);// Point 1 (Top)
	glVertex3f(-1.0f, 1.0f, 1.0f);// Point 2 (Top)
	glVertex3f(1.0f, 1.0f, 1.0f);// Point 3 (Top)
	glVertex3f(1.0f, 1.0f, -1.0f);// Point 4 (Top)
	// Bottom Face
	glNormal3f(0.0f, -1.0f, 0.0f);// Normal Pointing Down
	glVertex3f(-1.0f, -1.0f, -1.0f);// Point 1 (Bottom)
	glVertex3f(1.0f, -1.0f, -1.0f);// Point 2 (Bottom)
	glVertex3f(1.0f, -1.0f, 1.0f);// Point 3 (Bottom)
	glVertex3f(-1.0f, -1.0f, 1.0f);// Point 4 (Bottom)
	// Right face
	glNormal3f(1.0f, 0.0f, 0.0f);// Normal Pointing Right
	glVertex3f(1.0f, -1.0f, -1.0f);// Point 1 (Right)
	glVertex3f(1.0f, 1.0f, -1.0f);// Point 2 (Right)
	glVertex3f(1.0f, 1.0f, 1.0f);// Point 3 (Right)
	glVertex3f(1.0f, -1.0f, 1.0f);// Point 4 (Right)
	// Left Face
	glNormal3f(-1.0f, 0.0f, 0.0f);// Normal Pointing Left
	glVertex3f(-1.0f, -1.0f, -1.0f);// Point 1 (Left)
	glVertex3f(-1.0f, -1.0f, 1.0f);// Point 2 (Left)
	glVertex3f(-1.0f, 1.0f, 1.0f);// Point 3 (Left)
	glVertex3f(-1.0f, 1.0f, -1.0f);// Point 4 (Left)
	glEnd();

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor4f(0.2f, 0.65f, 0.3f, 0.35f);// Full Brightness, 50% Alpha ( NEW )
	glBegin(GL_QUADS);
	// Front Face
	glNormal3f(0.0f, 0.0f, 1.0f);// Normal Pointing Towards Viewer
	glVertex3f(-1.0f, -1.0f, 1.0f);// Point 1 (Front)
	glVertex3f(1.0f, -1.0f, 1.0f);// Point 2 (Front)
	glVertex3f(1.0f, 1.0f, 1.0f);// Point 3 (Front)
	glVertex3f(-1.0f, 1.0f, 1.0f);// Point 4 (Front)
	// Back Face
	glNormal3f(0.0f, 0.0f, -1.0f);// Normal Pointing Away From Viewer
	glVertex3f(-1.0f, -1.0f, -1.0f);// Point 1 (Back)
	glVertex3f(-1.0f, 1.0f, -1.0f);// Point 2 (Back)
	glVertex3f(1.0f, 1.0f, -1.0f);// Point 3 (Back)
	glVertex3f(1.0f, -1.0f, -1.0f);// Point 4 (Back)
	// Top Face
	glNormal3f(0.0f, 1.0f, 0.0f);// Normal Pointing Up
	glVertex3f(-1.0f, 1.0f, -1.0f);// Point 1 (Top)
	glVertex3f(-1.0f, 1.0f, 1.0f);// Point 2 (Top)
	glVertex3f(1.0f, 1.0f, 1.0f);// Point 3 (Top)
	glVertex3f(1.0f, 1.0f, -1.0f);// Point 4 (Top)
	// Bottom Face
	glNormal3f(0.0f, -1.0f, 0.0f);// Normal Pointing Down
	glVertex3f(-1.0f, -1.0f, -1.0f);// Point 1 (Bottom)
	glVertex3f(1.0f, -1.0f, -1.0f);// Point 2 (Bottom)
	glVertex3f(1.0f, -1.0f, 1.0f);// Point 3 (Bottom)
	glVertex3f(-1.0f, -1.0f, 1.0f);// Point 4 (Bottom)
	// Right face
	glNormal3f(1.0f, 0.0f, 0.0f);// Normal Pointing Right
	glVertex3f(1.0f, -1.0f, -1.0f);// Point 1 (Right)
	glVertex3f(1.0f, 1.0f, -1.0f);// Point 2 (Right)
	glVertex3f(1.0f, 1.0f, 1.0f);// Point 3 (Right)
	glVertex3f(1.0f, -1.0f, 1.0f);// Point 4 (Right)
	// Left Face
	glNormal3f(-1.0f, 0.0f, 0.0f);// Normal Pointing Left
	glVertex3f(-1.0f, -1.0f, -1.0f);// Point 1 (Left)
	glVertex3f(-1.0f, -1.0f, 1.0f);// Point 2 (Left)
	glVertex3f(-1.0f, 1.0f, 1.0f);// Point 3 (Left)
	glVertex3f(-1.0f, 1.0f, -1.0f);// Point 4 (Left)
	glEnd();

	glPopAttrib();
#endif
}

bool ARDrawingContext::drawMesh() {
	cout << "display" << endl;
	/*glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	 glClearColor(0.0f, 0.0f, 0.0f, 1.0f);


	 // Set lookat point
	 glLoadIdentity();
	 glTranslatef(0,0,-5.0f);
	 glRotatef(xAngle, 1.0F, 0, 0);
	 glRotatef(yAngle, 0, 1.0F, 0);*/
	//glScalef(12.2f,12.2f,12.2f);

	glShadeModel(GL_SMOOTH);
	glDisable(GL_CULL_FACE);

	//glDisable(GL_CULL_FACE);
	// NOTE: we could use display lists here.  That would speed things
	// up which the user is rotating the mesh.  However, since speed isn't
	// a bit issue, I didn't use them.
	if (g_pProgMesh) {
		cout << "going into if branch" << endl;
		// Make everything grey
		//glColor3f(128, 128, 128);
		//cout<<g_pProgMesh->numTris()<<endl;
		Mesh& m = *g_pProgMesh->_mesh;
		vector<vertex>& vec = m._vlist;
		if (true) {
			GLfloat *vert = new GLfloat[vec.size() * 3];
			GLfloat *norm = new GLfloat[vec.size() * 3];
			GLushort *vertElem = new GLushort[m._plist.size() * 3];
			for (int i = 0; i < vec.size(); i++) {
				vert[i * 3] = vec[i]._myVertex.x;
				vert[i * 3 + 1] = vec[i]._myVertex.y;
				vert[i * 3 + 2] = vec[i]._myVertex.z;

				norm[i * 3] = vec[i]._vertexNormal.x;
				norm[i * 3 + 1] = vec[i]._vertexNormal.y;
				norm[i * 3 + 2] = vec[i]._vertexNormal.z;
			}
			/*cout<<"vert: "<<endl;
			 for(int i=0;i<vec.size();i++){
			 cout<<vert[i*3]<<"  "<<vert[i*3+1]<<"  "<<vert[i*3+2]<<"  "<<norm[i*3]<<"  "<<norm[i*3+1]<<"  "<<norm[i*3+2]<<endl;
			 }*/
			for (int i = 0; i < m._plist.size(); i++) {
				vertElem[i * 3] = m._plist[i]._vert1;
				vertElem[i * 3 + 1] = m._plist[i]._vert2;
				vertElem[i * 3 + 2] = m._plist[i]._vert3;
			}
			//cout<<"vertElem: "<<endl;
			/*for(int i=0;i<m._plist.size();i++){
			 cout<<vertElem[i*3]<<"  "<<vertElem[i*3+1]<<"  "<<vertElem[i*3+2]<<endl;
			 }*/

			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_NORMAL_ARRAY);
			glVertexPointer(3, GL_FLOAT, 0, vert);
			glNormalPointer(GL_FLOAT, 0, norm);
			glDisable(GL_CULL_FACE);
			glDrawElements(GL_TRIANGLES, m._plist.size() * 3, GL_UNSIGNED_SHORT, vertElem);
			cout << "num triangles: " << g_pProgMesh->numTris() << "  " << m._plist.size() << endl;
			/*cout<<"vertices"<<endl;
			 vector<vertex>& v=g_pProgMesh->_mesh->_vlist;
			 for(int i=0;i<v.size();i++)
			 {
			 cout<<v[i]._v[0]<<"  "<<v[i]._v[1]<<"  "<<v[i]._v[2]<<endl;
			 }
			 cout<<"triangles"<<endl;
			 for(int i=0;i<g_pProgMesh->numTris()*3;i++){
			 cout<<vertices[i*3]<<vertices[i*3+1]<<vertices[i*3+2]<<endl;
			 }*/
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_NORMAL_ARRAY);
		}
		else {
			GLfloat *vert = new GLfloat[vec.size() * 3];
			GLfloat *norm = new GLfloat[vec.size() * 3];
			GLushort *vertElem = new GLushort[m._plist.size() * 6];
			for (int i = 0; i < vec.size(); i++) {
				vert[i * 3] = vec[i]._myVertex.x;
				vert[i * 3 + 1] = vec[i]._myVertex.y;
				vert[i * 3 + 2] = vec[i]._myVertex.z;

				norm[i * 3] = -vec[i]._vertexNormal.x;
				norm[i * 3 + 1] = -vec[i]._vertexNormal.y;
				norm[i * 3 + 2] = -vec[i]._vertexNormal.z;
			}
			/*cout<<"vert: "<<endl;
			 for(int i=0;i<vec.size();i++){
			 cout<<vert[i*3]<<"  "<<vert[i*3+1]<<"  "<<vert[i*3+2]<<"  "<<norm[i*3]<<"  "<<norm[i*3+1]<<"  "<<norm[i*3+2]<<endl;
			 }*/
			for (int i = 0; i < m._plist.size(); i++) {
				vertElem[i * 6] = m._plist[i]._vert1;
				vertElem[i * 6 + 1] = m._plist[i]._vert2;
				vertElem[i * 6 + 2] = m._plist[i]._vert3;
				vertElem[i * 6 + 3] = m._plist[i]._vert1;
				vertElem[i * 6 + 4] = m._plist[i]._vert2;
				vertElem[i * 6 + 5] = m._plist[i]._vert3;
			}
			//cout<<"vertElem: "<<endl;
			/*for(int i=0;i<m._plist.size();i++){
			 cout<<vertElem[i*3]<<"  "<<vertElem[i*3+1]<<"  "<<vertElem[i*3+2]<<endl;
			 }*/

			glEnableClientState(GL_VERTEX_ARRAY);
			//glEnableClientState(GL_NORMAL_ARRAY);
			glVertexPointer(3, GL_FLOAT, 0, vert);
			//glNormalPointer(GL_FLOAT,0,norm);
			glDrawElements(GL_LINES, m._plist.size() * 6, GL_UNSIGNED_SHORT, vertElem);
			cout << "num triangles: " << g_pProgMesh->numTris() << "  " << m._plist.size() << endl;
			/*cout<<"vertices"<<endl;
			 vector<vertex>& v=g_pProgMesh->_mesh->_vlist;
			 for(int i=0;i<v.size();i++)
			 {
			 cout<<v[i]._v[0]<<"  "<<v[i]._v[1]<<"  "<<v[i]._v[2]<<endl;
			 }
			 cout<<"triangles"<<endl;
			 for(int i=0;i<g_pProgMesh->numTris()*3;i++){
			 cout<<vertices[i*3]<<vertices[i*3+1]<<vertices[i*3+2]<<endl;
			 }*/
			glDisableClientState(GL_VERTEX_ARRAY);
			//glDisableClientState(GL_NORMAL_ARRAY);
		}
	}

	//glEnable(GL_CULL_FACE);

	glEnable(GL_CULL_FACE);
	return true;
}
