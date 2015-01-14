package com.example.ar.opengl.geometry;


public class Matrix33
{

	//float[] data = new float[9];
	public float[][] mat = new float[3][3];
	public Matrix33(Matrix33 m)
	{
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				mat[i][j]=m.mat[i][j];
			}
		}
	}

	public Matrix33() {
		// TODO Auto-generated constructor stub
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				mat[i][j]=0;
			}
		}
	}
	public static Matrix33 identity()
	{
		Matrix33 eye=new Matrix33();
		  
		  for (int i=0;i<3; i++)
		    for (int j=0;j<3;j++)
		      eye.mat[i][j] = i == j ? 1 : 0;
		  
		  return eye;
	}
	public Matrix33 getTransposed()
	{
		Matrix33 t=new Matrix33();
		  
		  for (int i=0;i<3; i++)
		    for (int j=0;j<3;j++)
		      t.mat[i][j] = mat[j][i];
		  
		  return t;
	}
}