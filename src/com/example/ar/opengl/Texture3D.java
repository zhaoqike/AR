package com.example.ar.opengl;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Vector;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.R.string;
import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.opengl.GLSurfaceView;
import android.opengl.GLUtils;
import android.opengl.GLSurfaceView.Renderer;
import android.os.Bundle;
import android.util.Log;
import android.view.GestureDetector;
import android.view.GestureDetector.OnGestureListener;
import android.view.SurfaceHolder;
import android.view.animation.Transformation;
import android.view.MotionEvent;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.*;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.objdetect.CascadeClassifier;

import com.example.ar.opengl.geometry.GLTransformation;
import com.example.ar.opengl.geometry.Matrix44;
import com.example.ar.opengl.geometry.Vector3;


/**
 * Description:
 * <br/>site: <a href="http://www.crazyit.org">crazyit.org</a>
 * <br/>Copyright (C), 2001-2014, Yeeku.H.Lee
 * <br/>This program is protected by copyright laws.
 * <br/>Program Name:
 * <br/>Date:
 * @author  Yeeku.H.Lee kongyeeku@163.com
 * @version  1.0
 */
public class Texture3D extends Activity implements OnGestureListener, SurfaceHolder.Callback
{
	// 定义旋转角度
	private float anglex = 0f;
	private float angley = 0f;
	static final float ROTATE_FACTOR = 60;
	private static final String TAG = "texture3d";
	// 定义手势检测器实例
	GestureDetector detector;
	int option=1;

	public float w=0.5f;
	public float h=0.5f;

	Vector<GLTransformation> pose3ds;
	Vector3 center;
	float centerx;
	float centery;
	float centerz;
	ReadText reader=new ReadText();

	CameraCalibration calibration;
	GLTransformation patternPose=new GLTransformation();
	int index=0;


	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {

		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS:
			{
				Log.i(TAG, "OpenCV loaded successfully");

				/* Now enable camera view to start receiving frames */
				calibration=new CameraCalibration(526.58037684199849f, 524.65577209994706f, 318.41744018680112f, 202.96659047014398f);
				Log.e(TAG, "OpenCV loaded successfully");
				Log.e(TAG, "calibration successfully");
				//mOpenCvCameraView.setOnTouchListener(Puzzle15Activity.this);
				//mOpenCvCameraView.enableView();
				
				String filePath="";
				if(option==1)
				{
					filePath="sdcard/out1";
				}
				else if(option==2){
					filePath="sdcard/out2";
				}
				pose3ds=reader.readPmatsFile(filePath);
				center=reader.readCenterFile(filePath);
				if(pose3ds!=null){
					Log.e(TAG, "poses read success");
				}
				else {
					Log.e(TAG, "poses read failed");
				}
				if(center!=null){
					Log.e(TAG, "center read success");
					centerx=center.data[0];
					centery=center.data[1];
					centerz=center.data[2];
				}
				else {
					Log.e(TAG, "center read failed");
				}
			} break;
			default:
			{
				super.onManagerConnected(status);
			} break;
			}
		}
	};
	private SurfaceHolder surfaceHolder;

	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		// 创建一个GLSurfaceView，用于显示OpenGL绘制的图形
		GLSurfaceView glView = new GLSurfaceView(this);
		//GLSurfaceView glView = (GLSurfaceView)findViewById(R.id.glview);//new GLSurfaceView(this);
		// 创建GLSurfaceView的内容绘制器
		MyRenderer myRender = new MyRenderer(this);
		// 为GLSurfaceView设置绘制器
		glView.setRenderer(myRender);
		setContentView(glView);
		// 创建手势检测器
		detector = new GestureDetector(this , this);
		

		//surfaceHolder=glView.getHolder();//SurfaceHolder是SurfaceView的控制接口
        //surfaceHolder.addCallback(this); //因为这个类实现了SurfaceHolder.Callback接口，所以回调参数直接this
        //surfaceHolder.setFixedSize(320, 240);//显示的分辨率,不设置为视频默认
        //surfaceHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);//Surface类型
	}

	@Override
	public void onResume()
	{
		super.onResume();
		OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this, mLoaderCallback);
	}

	@Override
	public boolean onTouchEvent(MotionEvent me)
	{
		// 将该Activity上的触碰事件交给GestureDetector处理

		return detector.onTouchEvent(me);

	}

	@Override
	public boolean onFling(MotionEvent event1, MotionEvent event2,
			float velocityX, float velocityY)
	{
		velocityX = velocityX > 2000 ? 2000 : velocityX;
		velocityX = velocityX < -2000 ? -2000 : velocityX;
		velocityY = velocityY > 2000 ? 2000 : velocityY;
		velocityY = velocityY < -2000 ? -2000 : velocityY;
		// 根据横向上的速度计算沿Y轴旋转的角度
		angley += velocityX * ROTATE_FACTOR / 4000;
		// 根据纵向上的速度计算沿X轴旋转的角度
		anglex += velocityY * ROTATE_FACTOR / 4000;


		index++;

		if(index>=pose3ds.size())
		{
			index=0;
		}
		Log.e(TAG,""+index);

		return true;
	}

	@Override
	public boolean onDown(MotionEvent arg0)
	{
		return false;
	}

	@Override
	public void onLongPress(MotionEvent event)
	{
	}

	@Override
	public boolean onScroll(MotionEvent event1, MotionEvent event2,
			float distanceX, float distanceY)
	{
		return false;
	}

	@Override
	public void onShowPress(MotionEvent event)
	{
	}

	@Override
	public boolean onSingleTapUp(MotionEvent event)
	{
		return false;
	}

	public class MyRenderer implements Renderer
	{


		boolean m_isTextureInitialized=false;
		int[] m_backgroundTextureId=new int[1];
		int backgroundTexture;
		// 立方体的顶点座标（一共是36个顶点，组成12个三角形）
		private float[] cubeVertices = { -0.6f, -0.6f, -0.6f, -0.6f, 0.6f,
				-0.6f, 0.6f, 0.6f, -0.6f, 0.6f, 0.6f, -0.6f, 0.6f, -0.6f, -0.6f,
				-0.6f, -0.6f, -0.6f, -0.6f, -0.6f, 0.6f, 0.6f, -0.6f, 0.6f, 0.6f,
				0.6f, 0.6f, 0.6f, 0.6f, 0.6f, -0.6f, 0.6f, 0.6f, -0.6f, -0.6f,
				0.6f, -0.6f, -0.6f, -0.6f, 0.6f, -0.6f, -0.6f, 0.6f, -0.6f, 0.6f,
				0.6f, -0.6f, 0.6f, -0.6f, -0.6f, 0.6f, -0.6f, -0.6f, -0.6f, 0.6f,
				-0.6f, -0.6f, 0.6f, 0.6f, -0.6f, 0.6f, 0.6f, 0.6f, 0.6f, 0.6f,
				0.6f, 0.6f, -0.6f, 0.6f, 0.6f, -0.6f, -0.6f, 0.6f, 0.6f, -0.6f,
				-0.6f, 0.6f, -0.6f, -0.6f, 0.6f, 0.6f, -0.6f, 0.6f, 0.6f, 0.6f,
				0.6f, 0.6f, 0.6f, 0.6f, -0.6f, -0.6f, 0.6f, -0.6f, -0.6f, -0.6f,
				-0.6f, -0.6f, -0.6f, 0.6f, -0.6f, -0.6f, 0.6f, -0.6f, 0.6f, 0.6f,
				-0.6f, 0.6f, -0.6f, };
		// 定义立方体所需要的6个面（一共是12个三角形所需的顶点）
		private byte[] cubeFacets = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
				13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
				30, 31, 32, 33, 34, 35, };
		// 定义纹理贴图的座标数据
		private float[] cubeTextures = { 1.0000f, 1.0000f, 1.0000f, 0.0000f,
				0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 1.0000f, 1.0000f,
				1.0000f, 0.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 0.0000f,
				1.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 1.0000f, 0.0000f,
				1.0000f, 1.0000f, 1.0000f, 1.0000f, 0.0000f, 1.0000f, 0.0000f,
				0.0000f, 0.0000f, 0.0000f, 1.0000f, 0.0000f, 1.0000f, 1.0000f,
				1.0000f, 1.0000f, 0.0000f, 1.0000f, 0.0000f, 0.0000f, 0.0000f,
				0.0000f, 1.0000f, 0.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
				0.0000f, 1.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 1.0000f,
				0.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 0.0000f, 1.0000f,
				0.0000f, 0.0000f, 0.0000f, 0.0000f, 1.0000f };

		private Context context;
		private FloatBuffer cubeVerticesBuffer;
		private ByteBuffer cubeFacetsBuffer;
		private FloatBuffer cubeTexturesBuffer;
		// 定义本程序所使用的纹理
		private int texture;

		public MyRenderer(Context main)
		{
			this.context = main;
			// 将立方体的顶点位置数据数组包装成FloatBuffer;
			cubeVerticesBuffer = floatBufferUtil(cubeVertices);
			// 将立方体的6个面（12个三角形）的数组包装成ByteBuffer
			cubeFacetsBuffer = ByteBuffer.wrap(cubeFacets);
			// 将立方体的纹理贴图的座标数据包装成FloatBuffer
			cubeTexturesBuffer = floatBufferUtil(cubeTextures);
		}

		@Override
		public void onSurfaceCreated(GL10 gl, EGLConfig config)
		{
			// 关闭抗抖动
			gl.glDisable(GL10.GL_DITHER);
			// 设置系统对透视进行修正
			gl.glHint(GL10.GL_PERSPECTIVE_CORRECTION_HINT, GL10.GL_FASTEST);
			gl.glClearColor(0, 0, 0, 0);
			// 设置阴影平滑模式
			gl.glShadeModel(GL10.GL_SMOOTH);
			// 启用深度测试
			gl.glEnable(GL10.GL_DEPTH_TEST);
			// 设置深度测试的类型
			gl.glDepthFunc(GL10.GL_LEQUAL);
			// 启用2D纹理贴图
			gl.glEnable(GL10.GL_TEXTURE_2D);
			// 装载纹理
			//loadTexture(gl);
		}

		@Override
		public void onSurfaceChanged(GL10 gl, int width, int height)
		{
			// 设置3D视窗的大小及位置
			gl.glViewport((width-640)/2, (height-480)/2, 640, 480);
			// 将当前矩阵模式设为投影矩阵
			gl.glMatrixMode(GL10.GL_PROJECTION);
			// 初始化单位矩阵
			//gl.glLoadIdentity();
			// 计算透视视窗的宽度、高度比
			//float ratio = (float) width / height;
			// 调用此方法设置透视视窗的空间大小。
			//set frustum
			float w=640;
			float h=480;
			float znear = 1.0f;
			float zfar = 100.0f;
			float u0 = w / 2;
			float v0 = h / 2;
			float maxwh = Math.max(w, h);
			float fx = maxwh;
			float fy = maxwh;
			float top = v0*znear / fy;
			float bottom = (v0 - h)*znear / fy;
			float left = -u0*znear / fx;
			float right = (w - u0)*znear / fx;
			gl.glFrustumf(left, right, bottom, top, znear, zfar);
		}

		void drawCameraFrame(GL10 gl)                                  // Render background
		{
			// Initialize texture for background image

			if (!m_isTextureInitialized)
			{
				gl.glGenTextures(1, m_backgroundTextureId, 0);
				backgroundTexture=m_backgroundTextureId[0];
				gl.glBindTexture(GL10.GL_TEXTURE_2D, backgroundTexture);

				gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_MIN_FILTER, GL10.GL_LINEAR);
				gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_MAG_FILTER, GL10.GL_LINEAR);

				m_isTextureInitialized = true;
			}

			Bitmap bitmap = null;
			//bitmap = BitmapFactory.decodeResource(context.getResources(),R.drawable.a80);
			// 加载位图生成纹理
			GLUtils.texImage2D(GL10.GL_TEXTURE_2D, 0, bitmap, 0);


			/*m_backgroundImage = imageList[pindex];
			Size size = m_backgroundImage.size();
			if (size.width != 640 || size.height != 480)
			{
				//cout << "background image size error" << endl;
				return;
			}
			int w = m_backgroundImage.cols;
			int h = m_backgroundImage.rows;

			glPixelStorei(GL_PACK_ALIGNMENT, 1);
			glBindTexture(GL_TEXTURE_2D, m_backgroundTextureId);

			// Upload new texture data:
			if (m_backgroundImage.channels() == 3)
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, m_backgroundImage.data);*/
			/*else if (m_backgroundImage.channels() == 4)
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, m_backgroundImage.data);
			else if (m_backgroundImage.channels() == 1)
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, m_backgroundImage.data);*/

			float bgTextureVertices[] = { 0, 0, (float)w, 0, 0, (float)h, (float)w, (float)h };
			float bgTextureCoords[] = { 1, 0, 1, 1, 0, 0, 0, 1 };
			float proj[] = { 0, -2.f / w, 0, 0, -2.f / h, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };
			FloatBuffer projBuffer=floatBufferUtil(proj);
			gl.glMatrixMode(GL10.GL_PROJECTION);
			gl.glLoadMatrixf(projBuffer);

			gl.glMatrixMode(GL10.GL_MODELVIEW);
			gl.glLoadIdentity();

			gl.glEnable(GL10.GL_TEXTURE_2D);
			//gl.glPixelStoref(GL10.GL_PACK_ALIGNMENT, 1);
			gl.glBindTexture(GL10.GL_TEXTURE_2D, backgroundTexture);

			// Update attribute values.
			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glEnableClientState(GL10.GL_TEXTURE_COORD_ARRAY);

			FloatBuffer vertexBuffer=floatBufferUtil(bgTextureVertices);
			FloatBuffer coordsBuffer=floatBufferUtil(bgTextureCoords);
			gl.glVertexPointer(2, GL10.GL_FLOAT, 0, vertexBuffer);
			gl.glTexCoordPointer(2, GL10.GL_FLOAT, 0, coordsBuffer);

			gl.glColor4f(1, 1, 1, 1);
			gl.glDrawArrays(GL10.GL_TRIANGLE_STRIP, 0, 4);

			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glDisableClientState(GL10.GL_TEXTURE_COORD_ARRAY);
			gl.glDisable(GL10.GL_TEXTURE_2D);
		}
		public void drawQuad(GL10 gl){

			if (!m_isTextureInitialized)
			{
				gl.glGenTextures(1, m_backgroundTextureId, 0);
				backgroundTexture=m_backgroundTextureId[0];
				gl.glBindTexture(GL10.GL_TEXTURE_2D, backgroundTexture);

				gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_MIN_FILTER, GL10.GL_LINEAR);
				gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_MAG_FILTER, GL10.GL_LINEAR);

				m_isTextureInitialized = true;
			}

			Bitmap bitmap = null;
			/*if(option==1)
			{
			if(index==0)
			{
				bitmap = BitmapFactory.decodeResource(context.getResources(),
						R.drawable.a80);
			}
			if(index==1)
			{
				bitmap = BitmapFactory.decodeResource(context.getResources(),
						R.drawable.a81);
			}
			if(index==2)
			{
				bitmap = BitmapFactory.decodeResource(context.getResources(),
						R.drawable.a82);
			}
			if(index==3)
			{
				bitmap = BitmapFactory.decodeResource(context.getResources(),
						R.drawable.a83);
			}
			if(index==4)
			{
				bitmap = BitmapFactory.decodeResource(context.getResources(),
						R.drawable.a84);
			}
			}
			else if(option==2)
			{
				if(index==0)
				{
					bitmap = BitmapFactory.decodeResource(context.getResources(),
							R.drawable.a90);
				}
				if(index==1)
				{
					bitmap = BitmapFactory.decodeResource(context.getResources(),
							R.drawable.a91);
				}
				if(index==2)
				{
					bitmap = BitmapFactory.decodeResource(context.getResources(),
							R.drawable.a92);
				}
				if(index==3)
				{
					bitmap = BitmapFactory.decodeResource(context.getResources(),
							R.drawable.a93);
				}
				if(index==4)
				{
					bitmap = BitmapFactory.decodeResource(context.getResources(),
							R.drawable.a94);
				}
				if(index==5)
				{
					bitmap = BitmapFactory.decodeResource(context.getResources(),
							R.drawable.a95);
				}
			}*/

			// 加载位图生成纹理
			GLUtils.texImage2D(GL10.GL_TEXTURE_2D, 0, bitmap, 0);


			// 清除屏幕缓存和深度缓存
			/*gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);


			float proj[] = { 0, -2.f / w, 0, 0, -2.f / h, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };
			FloatBuffer projBuffer=floatBufferUtil(proj);
			gl.glMatrixMode(GL10.GL_PROJECTION);
			gl.glLoadMatrixf(projBuffer);
			// 设置当前矩阵模式为模型视图。
			gl.glMatrixMode(GL10.GL_MODELVIEW);
			gl.glLoadIdentity();
			// 把绘图中心移入屏幕2个单位
			gl.glTranslatef(0f, 0.0f, -2.0f);*/
			

			// 清除屏幕缓存和深度缓存
			gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);
			// 启用顶点座标数据
			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			// 启用贴图座标数组数据
			gl.glEnableClientState(GL10.GL_TEXTURE_COORD_ARRAY);  //①
			
			
			float proj[] = { 0, -2.f / 640.0f, 0, 0, -2.f / 480.0f, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };
			FloatBuffer projBuffer=floatBufferUtil(proj);
			gl.glMatrixMode(GL10.GL_PROJECTION);
			//gl.glLoadMatrixf(projBuffer);
			// 设置当前矩阵模式为模型视图。
			gl.glMatrixMode(GL10.GL_MODELVIEW);
			gl.glLoadIdentity();
			// 把绘图中心移入屏幕2个单位
			gl.glTranslatef(0f, 0.0f, -2.0f);

			float bgTextureCoords[] = {  1, 1, 0, 1, 0, 0, 1, 0};
			float bgTextureVertices[] = { 1, -1,-1, -1,-1, 1, 1, 1  };
			FloatBuffer vertexBuffer=floatBufferUtil(bgTextureVertices);
			FloatBuffer coordsBuffer=floatBufferUtil(bgTextureCoords);

			gl.glColor4f(1, 1, 1, 1);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glEnableClientState(GL10.GL_TEXTURE_COORD_ARRAY);
			gl.glEnable(GL10.GL_TEXTURE_2D);
			//gl.glPixelStoref(GL10.GL_PACK_ALIGNMENT, 1);
			gl.glBindTexture(GL10.GL_TEXTURE_2D, backgroundTexture);
			gl.glVertexPointer(2, GL10.GL_FLOAT, 0, vertexBuffer);
			gl.glTexCoordPointer(2, GL10.GL_FLOAT, 0, coordsBuffer);

			gl.glDrawArrays(GL10.GL_TRIANGLE_FAN, 0, 4);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glDisableClientState(GL10.GL_TEXTURE_COORD_ARRAY);
			gl.glDisable(GL10.GL_TEXTURE_2D);
		}

		public void drawCoordinateAxis(GL10 gl)
		{
			float lineX[] = { 0, 0, 0, 1, 0, 0 };
			float lineY[] = { 0, 0, 0, 0, 1, 0 };
			float lineZ[] = { 0, 0, 0, 0, 0, 1 };
			FloatBuffer lxBuffer=floatBufferUtil(lineX);
			FloatBuffer lyBuffer=floatBufferUtil(lineY);
			FloatBuffer lzBuffer=floatBufferUtil(lineZ);

			gl.glLineWidth(2);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, lxBuffer);
			gl.glDrawArrays(GL10.GL_LINES,0,2);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, lyBuffer);
			gl.glDrawArrays(GL10.GL_LINES,0,2);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, lzBuffer);
			gl.glDrawArrays(GL10.GL_LINES,0,2);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);

			/*glBegin(GL_LINES);

			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3fv(lineX);
			glVertex3fv(lineX + 3);

			glColor3f(0.0f, 1.0f, 0.0f);
			glVertex3fv(lineY);
			glVertex3fv(lineY + 3);

			glColor3f(0.0f, 0.0f, 1.0f);
			glVertex3fv(lineZ);
			glVertex3fv(lineZ + 3);

			glEnd();*/
		}


		public void drawCubeModel(GL10 gl)
		{
			float LightAmbient[] = { 0.25f, 0.25f, 0.25f, 1.0f };    // Ambient Light Values
			float LightDiffuse[] = { 0.1f, 0.1f, 0.1f, 1.0f };    // Diffuse Light Values
			float LightPosition[] = { 0.0f, 0.0f, 2.0f, 1.0f };    // Light Position

			FloatBuffer laBuffer=floatBufferUtil(LightAmbient);
			FloatBuffer ldBuffer=floatBufferUtil(LightDiffuse);
			FloatBuffer lpBuffer=floatBufferUtil(LightPosition);

			//glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_POLYGON_BIT);

			gl.glColor4f(0.2f, 0.35f, 0.3f, 0.75f);         // Full Brightness, 50% Alpha ( NEW )
			gl.glBlendFunc(GL10.GL_ONE, GL10.GL_ONE_MINUS_SRC_ALPHA);       // Blending Function For Translucency Based On Source Alpha 
			gl.glEnable(GL10.GL_BLEND);

			gl.glShadeModel(GL10.GL_SMOOTH);

			gl.glEnable(GL10.GL_LIGHTING);
			gl.glDisable(GL10.GL_LIGHT0);
			gl.glEnable(GL10.GL_LIGHT1);
			gl.glLightfv(GL10.GL_LIGHT1, GL10.GL_AMBIENT, laBuffer);
			gl.glLightfv(GL10.GL_LIGHT1, GL10.GL_DIFFUSE, ldBuffer);
			gl.glLightfv(GL10.GL_LIGHT1, GL10.GL_POSITION, lpBuffer);
			gl.glEnable(GL10.GL_COLOR_MATERIAL);

			gl.glScalef(0.25f, 0.25f, 0.25f);
			gl.glTranslatef(0, 0, 1);

			//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			//glBegin(GL_QUADS);
			// Front Face
			/*glNormal3f(0.0f, 0.0f, 1.0f);    // Normal Pointing Towards Viewer
			glVertex3f(-1.0f, -1.0f, 1.0f);  // Point 1 (Front)
			glVertex3f(1.0f, -1.0f, 1.0f);  // Point 2 (Front)
			glVertex3f(1.0f, 1.0f, 1.0f);  // Point 3 (Front)
			glVertex3f(-1.0f, 1.0f, 1.0f);  // Point 4 (Front)*/

			//gles
			float q3front[] = {
					-1.0f, -1.0f, 1.0f,
					1.0f, -1.0f, 1.0f,
					1.0f, 1.0f, 1.0f,
					-1.0f, 1.0f, 1.0f
			};

			FloatBuffer frontBuffer=floatBufferUtil(q3front);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, frontBuffer);
			gl.glDrawArrays(GL10.GL_TRIANGLE_FAN,0,4);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);

			// Back Face
			/*glNormal3f(0.0f, 0.0f, -1.0f);    // Normal Pointing Away From Viewer
			glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Back)
			glVertex3f(-1.0f, 1.0f, -1.0f);  // Point 2 (Back)
			glVertex3f(1.0f, 1.0f, -1.0f);  // Point 3 (Back)
			glVertex3f(1.0f, -1.0f, -1.0f);  // Point 4 (Back)*/

			//gles
			float q3back[] = {
					-1.0f, -1.0f, -1.0f,
					-1.0f, 1.0f, -1.0f,
					1.0f, 1.0f, -1.0f,
					1.0f, -1.0f, -1.0f
			};

			FloatBuffer backBuffer=floatBufferUtil(q3back);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, backBuffer);
			gl.glDrawArrays(GL10.GL_TRIANGLE_FAN,0,4);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);


			// Top Face
			/*glNormal3f(0.0f, 1.0f, 0.0f);    // Normal Pointing Up
			glVertex3f(-1.0f, 1.0f, -1.0f);  // Point 1 (Top)
			glVertex3f(-1.0f, 1.0f, 1.0f);  // Point 2 (Top)
			glVertex3f(1.0f, 1.0f, 1.0f);  // Point 3 (Top)
			glVertex3f(1.0f, 1.0f, -1.0f);  // Point 4 (Top)*/

			//gles
			float q3top[] = {
					-1.0f, 1.0f, -1.0f,
					-1.0f, 1.0f, 1.0f,
					1.0f, 1.0f, 1.0f,
					1.0f, 1.0f, -1.0f
			};

			FloatBuffer topBuffer=floatBufferUtil(q3top);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, topBuffer);
			gl.glDrawArrays(GL10.GL_TRIANGLE_FAN,0,4);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);



			// Bottom Face
			/*glNormal3f(0.0f, -1.0f, 0.0f);    // Normal Pointing Down
			glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Bottom)
			glVertex3f(1.0f, -1.0f, -1.0f);  // Point 2 (Bottom)
			glVertex3f(1.0f, -1.0f, 1.0f);  // Point 3 (Bottom)
			glVertex3f(-1.0f, -1.0f, 1.0f);  // Point 4 (Bottom)*/

			//gles
			float q3bottom[] = {
					-1.0f, -1.0f, -1.0f,
					1.0f, -1.0f, -1.0f,
					1.0f, -1.0f, 1.0f,
					-1.0f, -1.0f, 1.0f
			};

			FloatBuffer bottomBuffer=floatBufferUtil(q3bottom);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, bottomBuffer);
			gl.glDrawArrays(GL10.GL_TRIANGLE_FAN,0,4);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);


			// Right face
			/*glNormal3f(1.0f, 0.0f, 0.0f);    // Normal Pointing Right
			glVertex3f(1.0f, -1.0f, -1.0f);  // Point 1 (Right)
			glVertex3f(1.0f, 1.0f, -1.0f);  // Point 2 (Right)
			glVertex3f(1.0f, 1.0f, 1.0f);  // Point 3 (Right)
			glVertex3f(1.0f, -1.0f, 1.0f);  // Point 4 (Right)*/

			//gles
			float q3right[] = {
					1.0f, -1.0f, -1.0f,
					1.0f, 1.0f, -1.0f,
					1.0f, 1.0f, 1.0f,
					1.0f, -1.0f, 1.0f
			};

			FloatBuffer rightBuffer=floatBufferUtil(q3right);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, rightBuffer);
			gl.glDrawArrays(GL10.GL_TRIANGLE_FAN,0,4);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);


			// Left Face
			/*glNormal3f(-1.0f, 0.0f, 0.0f);    // Normal Pointing Left
			glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Left)
			glVertex3f(-1.0f, -1.0f, 1.0f);  // Point 2 (Left)
			glVertex3f(-1.0f, 1.0f, 1.0f);  // Point 3 (Left)
			glVertex3f(-1.0f, 1.0f, -1.0f);  // Point 4 (Left)*/


			//gles
			float q3left[] = {
					-1.0f, -1.0f, -1.0f,
					-1.0f, -1.0f, 1.0f,
					-1.0f, 1.0f, 1.0f,
					-1.0f, 1.0f, -1.0f
			};

			FloatBuffer leftBuffer=floatBufferUtil(q3left);

			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, leftBuffer);
			gl.glDrawArrays(GL10.GL_TRIANGLE_FAN,0,4);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);


			//glEnd();

			/*glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glColor4f(0.2f, 0.65f, 0.3f, 0.35f); // Full Brightness, 50% Alpha ( NEW )
			glBegin(GL_QUADS);
			// Front Face
			glNormal3f(0.0f, 0.0f, 1.0f);    // Normal Pointing Towards Viewer
			glVertex3f(-1.0f, -1.0f, 1.0f);  // Point 1 (Front)
			glVertex3f(1.0f, -1.0f, 1.0f);  // Point 2 (Front)
			glVertex3f(1.0f, 1.0f, 1.0f);  // Point 3 (Front)
			glVertex3f(-1.0f, 1.0f, 1.0f);  // Point 4 (Front)
			// Back Face
			glNormal3f(0.0f, 0.0f, -1.0f);    // Normal Pointing Away From Viewer
			glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Back)
			glVertex3f(-1.0f, 1.0f, -1.0f);  // Point 2 (Back)
			glVertex3f(1.0f, 1.0f, -1.0f);  // Point 3 (Back)
			glVertex3f(1.0f, -1.0f, -1.0f);  // Point 4 (Back)
			// Top Face
			glNormal3f(0.0f, 1.0f, 0.0f);    // Normal Pointing Up
			glVertex3f(-1.0f, 1.0f, -1.0f);  // Point 1 (Top)
			glVertex3f(-1.0f, 1.0f, 1.0f);  // Point 2 (Top)
			glVertex3f(1.0f, 1.0f, 1.0f);  // Point 3 (Top)
			glVertex3f(1.0f, 1.0f, -1.0f);  // Point 4 (Top)
			// Bottom Face
			glNormal3f(0.0f, -1.0f, 0.0f);    // Normal Pointing Down
			glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Bottom)
			glVertex3f(1.0f, -1.0f, -1.0f);  // Point 2 (Bottom)
			glVertex3f(1.0f, -1.0f, 1.0f);  // Point 3 (Bottom)
			glVertex3f(-1.0f, -1.0f, 1.0f);  // Point 4 (Bottom)
			// Right face
			glNormal3f(1.0f, 0.0f, 0.0f);    // Normal Pointing Right
			glVertex3f(1.0f, -1.0f, -1.0f);  // Point 1 (Right)
			glVertex3f(1.0f, 1.0f, -1.0f);  // Point 2 (Right)
			glVertex3f(1.0f, 1.0f, 1.0f);  // Point 3 (Right)
			glVertex3f(1.0f, -1.0f, 1.0f);  // Point 4 (Right)
			// Left Face
			glNormal3f(-1.0f, 0.0f, 0.0f);    // Normal Pointing Left
			glVertex3f(-1.0f, -1.0f, -1.0f);  // Point 1 (Left)
			glVertex3f(-1.0f, -1.0f, 1.0f);  // Point 2 (Left)
			glVertex3f(-1.0f, 1.0f, 1.0f);  // Point 3 (Left)
			glVertex3f(-1.0f, 1.0f, -1.0f);  // Point 4 (Left)
			glEnd();

			glPopAttrib();*/
		}


		void drawAugmentedScene(GL10 gl)
		{
			// Init augmentation projection
			/*Matrix44 projectionMatrix = new Matrix44();
			int w = 640;
			int h = 480;
			buildProjectionMatrix(calibration, w, h, projectionMatrix);

			gl.glMatrixMode(gl.GL_PROJECTION);
			float[] data=new float[16];
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<4;j++)
				{
					data[i*4+j]=projectionMatrix.mat[i][j];
				}
			}
			FloatBuffer dataBuffer=floatBufferUtil(data);
			gl.glLoadMatrixf(dataBuffer);*/

			gl.glMatrixMode(gl.GL_MODELVIEW);
			gl.glLoadIdentity();

			if (true)
			{
				// Set the pattern transformation
				patternPose = pose3ds.elementAt(index);
				Matrix44 glMatrix = patternPose.getMat44();
				float[] glPose=patternPose.GLPose;
				float[] mdata=new float[16];
				for(int i=0;i<4;i++)
				{
					for(int j=0;j<4;j++)
					{
						mdata[i*4+j]=glMatrix.mat[i][j];
					}
				}
				String indexstring="";
				indexstring+=index;
				indexstring+="\n";
				String mdatastring="";
				for(int i=0;i<16;i++)
				{
					mdatastring+=mdata[i]+"  ";
				}
				mdatastring+="\n";
				
				String glPosestringString="";
				for(int i=0;i<16;i++)
				{
					glPosestringString+=patternPose.GLPose[i]+"  ";
				}
				glPosestringString+="\n";
				
				appendMethod("sdcard/glpose.txt", indexstring+mdatastring+glPosestringString);
				
				Log.e(TAG, mdatastring);
				FloatBuffer mdataBuffer=floatBufferUtil(mdata);
				FloatBuffer glposeBuffer=floatBufferUtil(glPose);
				gl.glLoadMatrixf(glposeBuffer);
				gl.glTranslatef(centerx, -centery, -centerz);
				//gl.glTranslatef(0, 0, -centerz);
				gl.glScalef(15, 15, 15);

				// Render model
				drawCoordinateAxis(gl);
				drawCubeModel(gl);
			}
		}
		
		public void appendMethod(String fileName, String content) {
	        try {
	            //打开一个写文件器，构造函数中的第二个参数true表示以追加形式写文件
	            FileWriter writer = new FileWriter(fileName, true);
	            writer.write(content);
	            writer.close();
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
	    }
		
		/*public Matrix44 transformAxis(Matrix44 m)
		{
			
		}*/

		void buildProjectionMatrix(CameraCalibration calibration, int screen_width, int screen_height, Matrix44 projectionMatrix)
		{
			Log.e(TAG, "begin build calibration");
			if(calibration==null){
				Log.e(TAG, "calibration null");
			}
			float nearPlane = 0.01f;  // Near clipping distance
			float farPlane = 100.0f;  // Far clipping distance

			// Camera parameters
			float f_x = calibration.fx(); // Focal length in x axis
			Log.e(TAG, ""+f_x);
			float f_y = calibration.fy(); // Focal length in y axis (usually the same?)
			float c_x = calibration.cx(); // Camera primary point x
			float c_y = calibration.cy(); // Camera primary point y

			projectionMatrix.mat[0][0] = -2.0f * f_x / screen_width;
			projectionMatrix.mat[0][1] = 0.0f;
			projectionMatrix.mat[0][2] = 0.0f;
			projectionMatrix.mat[0][3] = 0.0f;

			projectionMatrix.mat[1][0] = 0.0f;
			projectionMatrix.mat[1][1] = 2.0f * f_y / screen_height;
			projectionMatrix.mat[1][2] = 0.0f;
			projectionMatrix.mat[1][3] = 0.0f;

			projectionMatrix.mat[2][0] = 2.0f * c_x / screen_width - 1.0f;
			projectionMatrix.mat[2][1] = 2.0f * c_y / screen_height - 1.0f;
			projectionMatrix.mat[2][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
			projectionMatrix.mat[2][3] = -1.0f;

			projectionMatrix.mat[3][0] = 0.0f;
			projectionMatrix.mat[3][1] = 0.0f;
			projectionMatrix.mat[3][2] = -2.0f * farPlane * nearPlane / (farPlane - nearPlane);
			projectionMatrix.mat[3][3] = 0.0f;
		}


		public void onDrawFrame(GL10 gl)
		{
			//drawCameraFrame(gl);

			/*// 清除屏幕缓存和深度缓存
			gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);
			// 启用顶点座标数据
			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			// 启用贴图座标数组数据
			gl.glEnableClientState(GL10.GL_TEXTURE_COORD_ARRAY);  //①
			// 设置当前矩阵模式为模型视图。
			gl.glMatrixMode(GL10.GL_MODELVIEW);
			gl.glLoadIdentity();
			// 把绘图中心移入屏幕2个单位
			gl.glTranslatef(0f, 0.0f, -2.0f);*/
			gl.glDisable(gl.GL_DEPTH_TEST);
			drawQuad(gl);
			gl.glEnable(gl.GL_DEPTH_TEST);
			//gl.glFinish();
			// 旋转图形
			//gl.glRotatef(angley, 0, 1, 0);
			//gl.glRotatef(anglex, 1, 0, 0);

			//drawCoordinateAxis(gl);
			//drawCubeModel(gl);
			drawAugmentedScene(gl);
			/*Matrix44 projectionMatrix = new Matrix44();
			int w = 640;
			int h = 480;
			//buildProjectionMatrix(calibration, w, h, projectionMatrix);

			gl.glMatrixMode(gl.GL_PROJECTION);
			float[] data=new float[16];
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<4;j++)
				{
					data[i*4+j]=projectionMatrix.mat[i][j];
				}
			}
			FloatBuffer dataBuffer=floatBufferUtil(data);
			//gl.glLoadMatrixf(dataBuffer);

			gl.glMatrixMode(gl.GL_MODELVIEW);
			gl.glLoadIdentity();*/

			/*gl.glMatrixMode(gl.GL_MODELVIEW);
			gl.glLoadIdentity();
			gl.glTranslatef(0f, 0.0f, -3.0f);
			drawCoordinateAxis(gl);*/
			// 设置顶点的位置数据
			//gl.glVertexPointer(3, GL10.GL_FLOAT, 0, cubeVerticesBuffer);
			// 设置贴图的座标数据
			//gl.glTexCoordPointer(2, GL10.GL_FLOAT, 0, cubeTexturesBuffer);//②

			// 执行纹理贴图
			//gl.glBindTexture(GL10.GL_TEXTURE_2D, texture);  //③
			// 按cubeFacetsBuffer指定的面绘制三角形

			//gl.glDrawElements(GL10.GL_TRIANGLES, cubeFacetsBuffer.remaining(), GL10.GL_UNSIGNED_BYTE, cubeFacetsBuffer);
			// 绘制结束
			gl.glFinish();
			// 禁用顶点、纹理座标数组
			//gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);
			//gl.glDisableClientState(GL10.GL_TEXTURE_COORD_ARRAY);
			// 递增角度值以便每次以不同角度绘制
		}

		/*private void loadTexture(GL10 gl)
		{
			Bitmap bitmap = null;
			try
			{
				// 加载位图
				bitmap = BitmapFactory.decodeResource(context.getResources(),
						R.drawable.sand);
				int[] textures = new int[1];
				// 指定生成N个纹理（第一个参数指定生成1个纹理），
				// textures数组将负责存储所有纹理的代号。
				gl.glGenTextures(1, textures, 0);
				// 获取textures纹理数组中的第一个纹理
				texture = textures[0];
				// 通知OpenGL将texture纹理绑定到GL10.GL_TEXTURE_2D目标中
				gl.glBindTexture(GL10.GL_TEXTURE_2D, texture);
				// 设置纹理被缩小（距离视点很远时被缩小）时候的滤波方式
				gl.glTexParameterf(GL10.GL_TEXTURE_2D,
						GL10.GL_TEXTURE_MIN_FILTER, GL10.GL_NEAREST);
				// 设置纹理被放大（距离视点很近时被方法）时候的滤波方式
				gl.glTexParameterf(GL10.GL_TEXTURE_2D,
						GL10.GL_TEXTURE_MAG_FILTER, GL10.GL_LINEAR);
				// 设置在横向、纵向上都是平铺纹理
				gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_WRAP_S,
						GL10.GL_REPEAT);
				gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_WRAP_T,
						GL10.GL_REPEAT);
				// 加载位图生成纹理
				GLUtils.texImage2D(GL10.GL_TEXTURE_2D, 0, bitmap, 0);
			}
			finally
			{
				// 生成纹理之后，回收位图
				if (bitmap != null)
					bitmap.recycle();
			}
		}*/
	}
	// 定义一个工具方法，将float[]数组转换为OpenGL ES所需的FloatBuffer
	private FloatBuffer floatBufferUtil(float[] arr)
	{
		FloatBuffer mBuffer;
		// 初始化ByteBuffer，长度为arr数组的长度*4，因为一个int占4个字节
		ByteBuffer qbb = ByteBuffer.allocateDirect(arr.length * 4);
		// 数组排列用nativeOrder
		qbb.order(ByteOrder.nativeOrder());
		mBuffer = qbb.asFloatBuffer();
		mBuffer.put(arr);
		mBuffer.position(0);
		return mBuffer;
	}
	


	@Override
	public void surfaceChanged(SurfaceHolder arg0, int arg1, int arg2, int arg3) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void surfaceCreated(SurfaceHolder arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void surfaceDestroyed(SurfaceHolder arg0) {
		// TODO Auto-generated method stub
		
	}
}