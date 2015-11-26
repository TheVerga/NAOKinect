using System;
using Microsoft.Kinect;

public class Matrix3x3
{
    public static int DIMENSION = 3;
    public double[,] matrix;

	public Matrix3x3(Matrix4 mat){

        matrix = new double[DIMENSION , DIMENSION];
        matrix[0,0] = mat.M11;
        matrix[0,1] = mat.M12;
        matrix[0,2] = mat.M13;
        matrix[1,0] = mat.M21;
        matrix[1,1] = mat.M22;
        matrix[1,2] = mat.M23;
        matrix[2,0] = mat.M31;
        matrix[2,1] = mat.M32;
        matrix[2,2] = mat.M33;
	}

    public Matrix3x3(){
        matrix = new double[DIMENSION, DIMENSION];
       for (int i = 0; i < DIMENSION; i++)
       {
           for (int j = 0; j < DIMENSION; j++)
           {
               matrix[i,j] = 0;
           }
       }
    }

    public static Matrix3x3 transpose(Matrix3x3 toTra)
    {
        Matrix3x3 result = new Matrix3x3();
        for (int i = 0; i < DIMENSION; i++)
        {
            for (int j = 0; j < DIMENSION; j++)
            {
                result.matrix[j,i] = toTra.matrix[i,j];
            }
        }
        return result;
    }

    public Matrix3x3 leftMul(Matrix3x3 mulby){
        Matrix3x3 result = new Matrix3x3();

        for(int i=0; i<DIMENSION; i++){
            for (int j = 0; j < DIMENSION; j++)
            {
                for (int k = 0; k < DIMENSION; k++)
                {
                    result.matrix[i,j] =  mulby.matrix[k,j] * this.matrix[i,k] ;
                }
            }
        }

        return result;
    }
}
