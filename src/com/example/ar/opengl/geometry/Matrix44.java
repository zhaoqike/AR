package com.example.ar.opengl.geometry;


public class Matrix44
{

	//float[] data = new float[16];
	public float[][] mat = new float[4][4];
	
	public Matrix44()
	{
		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
			{
				mat[i][j]=0f;
			}
		}
	}

	public Matrix44(Matrix44 m)
	{
		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
			{
				mat[i][j]=m.mat[i][j];
			}
		}
	}
	public Matrix44 getTransposed()
	{
		Matrix44 t=new Matrix44();
		  
		  for (int i=0;i<4; i++)
		    for (int j=0;j<4;j++)
		      t.mat[i][j] = mat[j][i];
		    
		  return t;
	}
	public Matrix44 getInvertedRT()
	{
		Matrix44 t = Matrix44.identity();
		  
		  for (int col=0;col<3; col++)
		  {
		    for (int row=0;row<3;row++)
		    { 
		      // Transpose rotation component (inversion)
		      t.mat[row][col] = mat[col][row];
		    }
		    
		    // Inverse translation component
		    t.mat[3][col] = - mat[3][col];
		  }
		  return t;
	}
	public static Matrix44 identity()
	{
		Matrix44 eye=new Matrix44();
		  
		  for (int i=0;i<4; i++)
		    for (int j=0;j<4;j++)
		      eye.mat[i][j] = i == j ? 1 : 0;
		  
		  return eye;
	}
}