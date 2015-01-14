package com.example.ar;

import java.util.List;
import java.util.ListIterator;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.Mat;














import com.example.ar.gljni.GLJNIActivity;
import com.example.ar.gljni.GLJNIView;

import android.app.Activity;
import android.app.Fragment;
import android.content.Intent;
import android.graphics.PixelFormat;
import android.hardware.Camera.Size;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SubMenu;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.FrameLayout;
import android.widget.Toast;

public class MainActivity extends Activity implements CvCameraViewListener2{

	private static final String TAG = "Main Activity";
	//int numPictures=2;
	String[] picturePath={
			"/sdcard/p4.jpg",
			"/sdcard/p2.jpg",
			"/sdcard/p3.jpg",
			"/sdcard/p4.jpg",
			"/sdcard/p5.jpg",
			"/sdcard/p6.jpg"
			};
	int pictureIndex=0;
	
	private boolean useMultiThread=false;
	
    private Mat mRgba;
    private Mat mGray;
    //private CameraBridgeViewBase mCameraView;
    private CameraView mCameraView;
    private GLJNIView glSurfaceView;
    private ARCubeRenderer mARRenderer;
    
    //menu resolution
    /*private List<Size> mResolutionList;
    private MenuItem[] mEffectMenuItems;
    private SubMenu mColorEffectsMenu;
    private MenuItem[] mResolutionMenuItems;
    private SubMenu mResolutionMenu;*/
    
    private SubMenu mPictureMenu;
    private MenuItem[] mPictureMenuItems;
    /*SubMenu preprocessSubMenu=null;
    SubMenu trackerSubMenu=null;
    
    final int PRE_FIRST=1;
    final int PRE_SECOND=2;
    final int PRE_START=3;
    final int PRE_STORE=4;
    
    final int TRK_LOD=5;
    final int TRK_START=6;*/
    
    
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    System.loadLibrary("ar");
                    mCameraView.enableFpsMeter();
                    mCameraView.enableView();
                    //mCameraView.setOnTouchListener( MainActivity.this );
                    Log.e(TAG,"train pattern start");
                    ARNativeLib.trainPatternNative(picturePath[pictureIndex]);
                    Log.e(TAG, "train pattern end");
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };
	
    
    
    public MainActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
        Log.i(TAG, "Instantiated new " + this.getClass()+"end");
    }


	@Override
	protected void onCreate(Bundle savedInstanceState) {
		Log.i(TAG, "called onCreate start");
		super.onCreate(savedInstanceState);
		//setContentView(R.layout.activity_main);

		/*if (savedInstanceState == null) {
			getFragmentManager().beginTransaction()
					.add(R.id.container, new PlaceholderFragment()).commit();
		}*/
		//Log.i(TAG, "called onCreatestart");
		
		getWindow().addFlags( WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON );
		FrameLayout layout = new FrameLayout(this);
        layout.setLayoutParams(new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT));
        setContentView(layout);
        
        //mCameraView = new NativeCameraView(this,0);
        mCameraView=new CameraView(this, 0);
        //Size size=new Size(640, 480);
        mCameraView.setResolution(640,480);
        mCameraView.setCvCameraViewListener(this);
        mCameraView.setLayoutParams(new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT));
        layout.addView(mCameraView);
        
        glSurfaceView = new GLJNIView(this);
        glSurfaceView.setHandler(new Handler(){
            @Override
            public void handleMessage(Message msg) {
                super.handleMessage(msg);
                
                // œ‘ æfps
                //mTextView.setText("fps:"+msg.what);
            }
        }
        );
        //glSurfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        glSurfaceView.getHolder().setFormat(PixelFormat.TRANSLUCENT);
        
        glSurfaceView.setZOrderOnTop(true);
        glSurfaceView.setLayoutParams(new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT));
        
        //mARRenderer = new ARCubeRenderer();

        //glSurfaceView.setRenderer(mARRenderer);
        
        glSurfaceView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
        layout.addView(glSurfaceView);
        
        if(useMultiThread){
        	Intent intent=new Intent(this,NativeService.class);
        	startService(intent);
        }
	}
	
	@Override
    public void onPause()
    {
        super.onPause();
        if (mCameraView != null)
            mCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this, mLoaderCallback);
    }
    
    @Override
    public void onDestroy() {
        super.onDestroy();
        mCameraView.disableView();
    }

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {

		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		
		
		mPictureMenu = menu.addSubMenu("Pictures");

        mPictureMenuItems = new MenuItem[picturePath.length];

        for(int idx=0;idx<mPictureMenuItems.length;idx++){

            mPictureMenuItems[idx] = mPictureMenu.add(2, idx, Menu.NONE, picturePath[idx]);

         }


		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// Handle action bar item clicks here. The action bar will
		// automatically handle clicks on the Home/Up button, so long
		// as you specify a parent activity in AndroidManifest.xml.
		switch(item.getItemId())
		{
		case R.id.switch_picture:
			pictureIndex++;
			if(pictureIndex>=picturePath.length){
				pictureIndex=0;
			}
			ARNativeLib.trainPatternNative(picturePath[pictureIndex]);
			//setPreprocessMode();
			//Intent intentPre=new Intent(MainActivity.this,GLJNIActivity.class);
			//startActivity(intentPre);
			break;
		case R.id.store_error:
			ARNativeLib.storeError();
			break;
		}
		if (item.getGroupId() == 2)
        {
            int id = item.getItemId();
            pictureIndex=id;
            ARNativeLib.trainPatternNative(picturePath[pictureIndex]);
        }
		return super.onOptionsItemSelected(item);
	}

	



	@Override
	public void onCameraViewStarted(int width, int height) {
		// TODO Auto-generated method stub
		Log.i(TAG, "called camera start");
		mGray = new Mat();
        mRgba = new Mat();
        Log.i(TAG, "called camera end");
        //mCameraView.setResolution(640,480);
	}


	@Override
	public void onCameraViewStopped() {
		// TODO Auto-generated method stub
		mRgba.release();
		mGray.release();
	}


	@Override
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		// TODO Auto-generated method stub
		Log.i(TAG, "called frame start");
		mRgba = inputFrame.rgba();
		mGray = inputFrame.gray();
		/*try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}*/
        Log.i(TAG, "called frame end");
        if(useMultiThread){
        	ARNativeLib.trackPatternMultiThreadNative(mGray.getNativeObjAddr(), mRgba.getNativeObjAddr());
        }
        else{
        	ARNativeLib.trackPatternNative(mGray.getNativeObjAddr(), mRgba.getNativeObjAddr());
        }
		glSurfaceView.requestRender();
        Log.e(TAG, "on camera frame");
        return mRgba;
        
	}

	
}
