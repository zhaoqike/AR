package com.example.ar.opengl;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.util.Vector;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

import com.example.ar.opengl.geometry.GLTransformation;
import com.example.ar.opengl.geometry.Matrix33;
import com.example.ar.opengl.geometry.Vector3;

import android.util.Log;

public class ReadText {
	private static String TAG= "read text";
	public int n=0;


	public void Read(){
		// TODO Auto-generated method stub
		System.out.println("hello workd");
		Vector<GLTransformation> poses = readPmatsFile("D:/Temp/backup/918/out");
		Vector3 center = readCenterFile("D:/Temp/backup/918/out");
		if(poses==null){
			System.out.println("error read poses");
		}
		else{
			System.out.println("success read poses");
		}
		
		if(center==null){
			System.out.println("error read center");
		}
		else{
			System.out.println("success read center");
		}
	}


	/**
	 * 功能：Java读取txt文件的内容
	 * 步骤：1：先获得文件句柄
	 * 2：获得文件句柄当做是输入一个字节码流，需要对这个输入流进行读取
	 * 3：读取到输入流后，需要读取生成字节流
	 * 4：一行一行的输出。readline()。
	 * 备注：需要考虑的是异常情况
	 * @param filePath
	 */
	public static Vector<GLTransformation> readPmatsFile(String filePath){
		Vector<GLTransformation> result=new Vector<GLTransformation>();
		try {
			File file=new File(filePath+"/Pmats.txt");
			if(file.isFile() && file.exists()){ //判断文件是否存在
				InputStreamReader read = new InputStreamReader(
						new FileInputStream(file));//考虑到编码格式
				BufferedReader bufferedReader = new BufferedReader(read);
				String lineTxt = null;
				float lineNum=0;
				if((lineTxt = bufferedReader.readLine()) != null){
					lineNum=Float.parseFloat(lineTxt);
				}
				while((lineTxt = bufferedReader.readLine()) != null){
					String[] nums=lineTxt.split("  ");
					System.out.println("nums.length: "+nums.length);
					String lString="";
					for(int i=0;i<nums.length;i++)
					{
						lString+=nums[i];
					}
					System.out.println(lString);
					System.out.println(lineTxt);
					if(nums.length==12)
					{
						
						
										        
				       
				        
				        
				        
				        
				        
				        
						float[] val=new float[12];
						for(int i=0;i<12;i++)
						{
							val[i]=Float.parseFloat(nums[i]);
						}
						//to transformation
						Matrix33 r=new Matrix33();
						r.mat[0][0] = val[0];
						r.mat[0][1] = val[1];
						r.mat[0][2] = val[2];

						r.mat[1][0] = val[4];
						r.mat[1][1] = val[5];
						r.mat[1][2] = val[6];

						r.mat[2][0] = val[8];
						r.mat[2][1] = val[9];
						r.mat[2][2] = val[10];
						
						Mat rotMat=new Mat(3,3,CvType.CV_64FC1);
						rotMat.put(0, 0, (double)val[0]);
						rotMat.put(0, 1, (double)val[1]);
						rotMat.put(0, 2, (double)val[2]);
						
						rotMat.put(1, 0, (double)val[4]);
						rotMat.put(1, 1, (double)val[5]);
						rotMat.put(1, 2, (double)val[6]);
						
						rotMat.put(2, 0, (double)val[8]);
						rotMat.put(2, 1, (double)val[9]);
						rotMat.put(2, 2, (double)val[10]);
						
						MatOfDouble mRVec=new MatOfDouble();
						Calib3d.Rodrigues(rotMat, mRVec);
						
						//MatOfDouble rvec=new MatOfDouble(mRVec);
						double[] rVecArray=mRVec.toArray();
				        rVecArray[1] *= -1.0;
				        rVecArray[2] *= -1.0;
				        mRVec.fromArray(rVecArray);
				        
				        MatOfDouble mRotation=new MatOfDouble();
				        Calib3d.Rodrigues(mRVec, mRotation);
						

						Vector3 t=new Vector3();
						t.data[0] = val[3];
						t.data[1] = val[7];
						t.data[2] = val[11];
						
						Mat tMat=new Mat(1,3,CvType.CV_64FC1);
						tMat.put(0, 0, (double)val[3]);
						tMat.put(0, 1, (double)val[7]);
						tMat.put(0, 2, (double)val[11]);
						
						MatOfDouble mTVec=new MatOfDouble(tMat);
						
						double[] tVecArray = mTVec.toArray();
						
						float[] mGLPose=new float[16];
						 mGLPose[0]  =  (float)mRotation.get(0, 0)[0];
					        mGLPose[1]  =  (float)mRotation.get(1, 0)[0];
					        mGLPose[2]  =  (float)mRotation.get(2, 0)[0];
					        mGLPose[3]  =  0f;
					        mGLPose[4]  =  (float)mRotation.get(0, 1)[0];
					        mGLPose[5]  =  (float)mRotation.get(1, 1)[0];
					        mGLPose[6]  =  (float)mRotation.get(2, 1)[0];
					        mGLPose[7]  =  0f;
					        mGLPose[8]  =  (float)mRotation.get(0, 2)[0];
					        mGLPose[9]  =  (float)mRotation.get(1, 2)[0];
					        mGLPose[10] =  (float)mRotation.get(2, 2)[0];
					        mGLPose[11] =  0f;
					        mGLPose[12] =  (float)tVecArray[0];
					        mGLPose[13] = -(float)tVecArray[1];
					        mGLPose[14] = -(float)tVecArray[2];
					        mGLPose[15] =  1f;
					        
					        Log.e(TAG,""+mGLPose[12]+"  "+mGLPose[13]+"  "+mGLPose[14]);

						GLTransformation pose3d=new GLTransformation(r, t);
						pose3d.GLPose=mGLPose;
						pose3d = pose3d.getInverted();
						result.add(pose3d);
					}

				}
				read.close();
				if(result.size()==lineNum)
				{
					return result;
				}
			}else{
				System.out.println("找不到指定的文件");
			}
		} catch (Exception e) {
			System.out.println("读取文件内容出错");
			e.printStackTrace();
		}
		return null;

	}


	public static Vector3 readCenterFile(String filePath){
		try {
			Log.e("read text", "begin try");
			File file=new File(filePath+"/center.txt");
			if(file.isFile() && file.exists()){ //判断文件是否存在
				InputStreamReader read = new InputStreamReader(
						new FileInputStream(file));//考虑到编码格式
				BufferedReader bufferedReader = new BufferedReader(read);
				String lineTxt = null;
				if((lineTxt = bufferedReader.readLine()) != null){
					Log.e(TAG, "get num");
					float tmp=Float.parseFloat(lineTxt);
				}
				if((lineTxt = bufferedReader.readLine()) != null){
					Log.e(TAG, "get result");
					String[] nums=lineTxt.split("  ");
					System.out.println("nums.length: "+nums.length);
					String lString="";
					for(int i=0;i<nums.length;i++)
					{
						lString+=nums[i];
					}
					System.out.println(lString);
					System.out.println(lineTxt);
					if(nums.length==3)
					{
						float[] val=new float[12];
						for(int i=0;i<3;i++)
						{
							val[i]=Float.parseFloat(nums[i]);
						}
						Vector3 result=new Vector3(val[0],val[1],val[2]);
						return result;
					}

				}
				read.close();
				return null;
			}else{
				Log.e(TAG, "file not found");
				System.out.println("找不到指定的文件");
				return null;
			}
		} catch (Exception e) {
			Log.e(TAG, "read inside error");
			System.out.println("读取文件内容出错");
			e.printStackTrace();
		}
		return null;

	}

}
