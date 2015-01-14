package com.example.ar.opengl.geometry;


public class GLTransformation
{
	public GLTransformation()
	{

	}
	public GLTransformation(Matrix33 r, Vector3 t)
	{
		m_rotation=r;
		m_translation=t;
	}
	public GLTransformation(Matrix33 r, Vector3 t,float[] glpose)
	{
		m_rotation=r;
		m_translation=t;
		GLPose=glpose;
	}

	public Matrix33 r(){
		return m_rotation;
	}
	public Vector3  t(){
		return m_translation;
	}



	
	public Matrix44 getMat44(){
		Matrix44 res = Matrix44.identity();
		  
		  for (int col=0;col<3;col++)
		  {
		    for (int row=0;row<3;row++)
		    {
		      // Copy rotation component
		      res.mat[row][col] = m_rotation.mat[row][col];
		    }
		    
		    // Copy translation component
		    res.mat[3][col] = m_translation.data[col];
		  }
		  
		  return res;
	}

	public GLTransformation getInverted(){
		return new GLTransformation(m_rotation.getTransposed(), m_translation.neg(),GLPose); 
		//return new GLTransformation(Matrix33.identity(), m_translation.neg()); 
	}
	
	private	Matrix33 m_rotation;
	private Vector3  m_translation;
	public float[] GLPose=new float[16];
}