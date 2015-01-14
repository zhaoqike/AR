package com.example.ar;

import java.util.List;

import org.opencv.android.JavaCameraView;

import android.content.Context;
import android.hardware.Camera.Size;
import android.util.AttributeSet;

public class CameraView extends JavaCameraView {
	
	private static final String TAG = "CameraView";

	public CameraView(Context context, AttributeSet attrs) {
		super(context, attrs);
		// TODO Auto-generated constructor stub
	}
	
	
	public CameraView(Context context, int i) {
		super(context, i);
		// TODO Auto-generated constructor stub
	}


	public List<Size> getResolutionList() {
        return mCamera.getParameters().getSupportedPreviewSizes();
    }

    public void setResolution(Size resolution) {
        disconnectCamera();
        mMaxHeight = resolution.height;
        mMaxWidth = resolution.width;
        connectCamera(getWidth(), getHeight());
    }
    
    public void setResolution(int width,int height) {
        //disconnectCamera();
        mMaxHeight = height;
        mMaxWidth = width;
        //connectCamera(getWidth(), getHeight());
    }

    public Size getResolution() {
        return mCamera.getParameters().getPreviewSize();
    }

}
