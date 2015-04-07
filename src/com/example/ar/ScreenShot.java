package com.example.ar;

import java.io.File;
import java.io.FileNotFoundException;   
import java.io.FileOutputStream;   
import java.io.IOException;   

import android.R.string;
import android.app.Activity;   
import android.graphics.Bitmap;   
import android.graphics.Rect;   
import android.view.View;   


public class ScreenShot {   
	
	public static int index=1;

	public static String folder="sdcard/screenshot/";
	public static String filename="shot";
	// ��ȡָ��Activity�Ľ��������浽png�ļ�   
	private static Bitmap takeScreenShot(Activity activity){   


		//View������Ҫ��ͼ��View   
		View view = activity.getWindow().getDecorView();   
		view.setDrawingCacheEnabled(true);   
		view.buildDrawingCache();   
		Bitmap b1 = view.getDrawingCache();   


		//��ȡ״̬���߶�   
		Rect frame = new Rect();   
		activity.getWindow().getDecorView().getWindowVisibleDisplayFrame(frame);   
		int statusBarHeight = frame.top;   
		System.out.println(statusBarHeight);   

		//��ȡ��Ļ���͸�   
		int width = activity.getWindowManager().getDefaultDisplay().getWidth();   
		int height = activity.getWindowManager().getDefaultDisplay().getHeight();   


		//ȥ��������   
		//Bitmap b = Bitmap.createBitmap(b1, 0, 25, 320, 455);   
		Bitmap b = Bitmap.createBitmap(b1, 0, statusBarHeight, width, height - statusBarHeight);   
		view.destroyDrawingCache();   
		return b;   
	}   


	private static void savePic(Bitmap b, File filePath) { 
        FileOutputStream fos = null; 
        try { 
            fos = new FileOutputStream(filePath); 
            if (null != fos) { 
                b.compress(Bitmap.CompressFormat.PNG, 100, fos); 
                fos.flush(); 
                fos.close(); 
            } 
        } catch (FileNotFoundException e) { 
            // e.printStackTrace(); 
        } catch (IOException e) { 
            // e.printStackTrace(); 
        } 
    }   


	public static boolean isFolderExists(String strFolder)
	{
		File file = new File(strFolder);

		if (!file.exists())
		{
			if (file.mkdir())
			{
				return true;
			}
			else
				return false;
		}
		return true;
	}


	//�������   
	public static void shoot(Activity a, File filePath) { 
        if (filePath == null) { 
            return; 
        } 
        if (!filePath.getParentFile().exists()) { 
            filePath.getParentFile().mkdirs(); 
        } 
        ScreenShot.savePic(ScreenShot.takeScreenShot(a), filePath); 
    }    
} 