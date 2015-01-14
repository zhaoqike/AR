package com.example.ar.opengl.geometry;




public class Vector4
{

	public Vector4(Vector4 v)
	{
		for(int i=0;i<4;i++)
		{
			data[i]=v.data[i];
		}
	}
	float[] data = new float[4];
}