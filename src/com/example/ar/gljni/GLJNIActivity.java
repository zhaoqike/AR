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
 * author: mnorst@foxmail.com
 */

package com.example.ar.gljni;

import java.text.DecimalFormat;


import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.ViewGroup.LayoutParams;
import android.widget.TextView;

public class GLJNIActivity extends Activity {
	GLJNIView mView;
	TextView mTextView;

	@Override
	protected void onCreate(Bundle icicle) {
		super.onCreate(icicle);
		mView = new GLJNIView(getApplication());
		setContentView(mView);
		
		mView.setHandler(new Handler(){
            @Override
            public void handleMessage(Message msg) {
                super.handleMessage(msg);
                
                // œ‘ æfps
                mTextView.setText("fps:"+msg.what);
            }
        }
        );
		
		mTextView = new TextView(this);
		mTextView.setText("fps:0");
		addContentView(mTextView, new LayoutParams(LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT));
		
		GLJNILib.test();
	}

	@Override
	protected void onPause() {
		super.onPause();
		mView.onPause();
	}

	@Override
	protected void onResume() {
		super.onResume();
		mView.onResume();
	}
}
