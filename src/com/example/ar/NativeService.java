package com.example.ar;

import android.app.IntentService;
import android.content.Intent;
import android.util.Log;

public class NativeService extends IntentService {

	private static final String TAG = "NativeService";

	public NativeService(String name) {
		super(name);
		// TODO Auto-generated constructor stub
	}
	
	public NativeService()
	{
		this("native service");
	}

	@Override
	protected void onHandleIntent(Intent arg0) {
		// TODO Auto-generated method stub
		Log.e(TAG, "on handle intent");
		ARNativeLib.cycleProcessNative();
	}

	

}
