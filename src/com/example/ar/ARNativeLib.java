package com.example.ar;

public class ARNativeLib {
	
	static {
        System.loadLibrary("ar");
    }
	
	public static native void trainPatternNative(String path);
	public static native boolean trackPatternNative(long matAddrGr, long matAddrRgba);
	public static native void trackPatternMultiThreadNative(long matAddrGr, long matAddrRgba);
	
	
	//preprocess
	public static native void setFirstFlagNative();
	
	public static native void setSecondFlagNative();
	public static native void setStartPrepTrackFlagNative();
	public static native void storeMapNative();
	public static native int preprocessFrameNative(long matAddrGr, long matAddrRgba);
	
	
	//tracking
	public static native void setStartTrackFlagNative();
	public static native int trackingFrameNative(long matAddrGr, long matAddrRgba);
	public static native void loadMapNative();
	//common
	public static native void redirectStdOut();
	public static native void FindFeatures(long matAddrGr, long matAddrRgba);
	
	public static native int processFrameNative(long matAddrGr, long matAddrRgba);
	
	
	//test
	public static native int sendFrameToNative(long matAddrGr, long matAddrRgba);
	public static native void cycleProcessNative();

	public static native boolean getGLPoseNative(float[] val);
	public static native boolean getCenterNative(float[] c);
	
	public static native void storeError();
	
	public static native void showRects();
	public static native void showPoints();
	public static native void showTexts();
	
	
	public static native void printWarp();
	
	public static native void printTime();
}
