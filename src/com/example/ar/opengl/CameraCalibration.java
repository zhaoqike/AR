package com.example.ar.opengl;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;

public class CameraCalibration {
	public CameraCalibration() {
	}
	
	public CameraCalibration(float fx, float fy, float cx, float cy) {
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++){
				m_intrinsic.put(i, j, (double)0);
			}
		}
		m_intrinsic.put(0, 0, (double)fx);
		m_intrinsic.put(1, 1, (double)fy);
		m_intrinsic.put(0, 2, (double)cx);
		m_intrinsic.put(1, 2, (double)cy);
		
		for(int i=0;i<5;i++)
		{
			m_distortion.put(0, i, (double)0);
		}
	}
	
	public CameraCalibration(float fx, float fy, float cx, float cy, float[] distorsionCoeff)
	{
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++){
				m_intrinsic.put(i, j, (double)0);
			}
		}
		m_intrinsic.put(0, 0, (double)fx);
		m_intrinsic.put(1, 1, (double)fy);
		m_intrinsic.put(0, 2, (double)cx);
		m_intrinsic.put(1, 2, (double)cy);
		
		for(int i=0;i<5;i++)
		{
			m_distortion.put(0, i, (double)distorsionCoeff[i]);
		}
	}

	public void getMatrix34(float[][] cparam){

	}

	public Mat getIntrinsic(){
		return m_intrinsic;
	}
	public Mat getDistorsion(){
		return m_distortion;
	}

	public float fx() {
		return (float) m_intrinsic.get(0,0)[0];
	}
	public float fy() {
		return (float) m_intrinsic.get(1,1)[0];
	}

	public float cx() {
		return (float) m_intrinsic.get(0,2)[0];
	}
	public float cy() {
		return (float) m_intrinsic.get(1,2)[0];
	}

	private Mat m_intrinsic=new Mat(3, 3, CvType.CV_32FC1);
	private Mat m_distortion=new Mat(1, 5, CvType.CV_32FC1);
}
