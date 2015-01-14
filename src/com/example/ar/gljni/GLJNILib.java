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

//Wrapper for native library
public class GLJNILib {
	
	static {
		System.loadLibrary("ar");
	}

	/**
     * @param width the current view width
     * @param height the current view height
     */
	public static native void resize(int width, int height); 
	
	/**
	 * render 
	 */
    public static native void step();  
    
    /**
     * init
     */
    public static native void init();  
    
    /**
     * set the texture
     * @param texture	texture id
     */
    public static native void setTexture(int[] texture);
    
    public static native void test();
}
