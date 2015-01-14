package com.example.ar.opengl.geometry;


public class Vector3
{
	public float[] data = new float[3];
	
	public Vector3()
	{
		data[0]=data[1]=data[2]=0;
	}
	
	public Vector3(Vector3 v){
		for(int i=0;i<3;i++)
		{
			data[i]=v.data[i];
		}
	}
	public Vector3(float data0,float data1,float data2)
	{
		data[0]=data0;
		data[1]=data1;
		data[2]=data2;
	}
	public static Vector3 zero()
	{
		return new Vector3(0f,0f,0f);
	}
	public Vector3 neg()
	{
		Vector3 v = new Vector3(-data[0],-data[1],-data[2] );
		return v;
	}
}
