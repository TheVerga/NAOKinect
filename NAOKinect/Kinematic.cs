using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace NAOKinect
{
    class Kinematic
    {
        ///<summary>
        /// Receive a rotation matrix of type Matrix4
        /// Return a Vector3Float containing the rotation around the 3 axis
        ///</summary>
        public static Vecto3Float computeEulerFromMatrixXYZ(Matrix3x3 m)
        {
            Vecto3Float sol = new Vecto3Float();
            if (m.matrix[0, 2] < 1f)
            {
                if (m.matrix[0, 2] > -1f)
                {

                    sol.X = (float)Math.Atan2(-m.matrix[1, 2], m.matrix[2, 2]);
                    sol.Y = (float)Math.Asin(-(m.matrix[0, 2]));
                    sol.Z = (float)Math.Atan2(-m.matrix[0, 1], m.matrix[0, 0]);
                }
                else
                {
                    sol.X = -(float)Math.Atan2(m.matrix[1, 0], m.matrix[1, 1]);
                    sol.Y = -(float)(Math.PI) / 2;
                    sol.Z = 0f;
                }

            }
            else
            {
                sol.X = (float)Math.Atan2(m.matrix[1, 0], m.matrix[1, 1]);
                sol.Y = (float)(Math.PI) / 2;
                sol.Z = 0f;
            }
            return sol;
        }

        public static float getAngleNew(Vecto3Float vect1, Vecto3Float vect2)

        {
            vect1.normalize();
            vect2.normalize();

            if (vect1.magnitude() != 0.0 && vect2.magnitude() != 0.0)
            {
                double x = (double)vect1.cross(vect2).magnitude();
                double y = (double)vect1.dot(vect2);

                return atan2(x, y);
            }
            else
            {
                return 0f;
            }
        }

        public static float atan2(double x, double y)
        {
            return (float)Math.Atan2(x, y);
        }

        ///<summary>
        /// Receive three skeleton point (3D position of a joint).
        /// Compute two vector, return the angle between them.
        /// If something is wrong return 0
        ///</summary>
        public static float getAngle(SkeletonPoint a, SkeletonPoint b, SkeletonPoint c, bool sign)
        {
            Vecto3Float[] vector = new Vecto3Float[2];
            //vector[0] = new Vecto3Float(b.X - c.X, b.Y - c.Y, b.Z - c.Z);
            vector[0] = new Vecto3Float(c.X - b.X, c.Y - b.Y, c.Z - b.Z);
            vector[1] = new Vecto3Float(a.X - b.X, a.Y - b.Y, a.Z - b.Z);


            float v0_magnitude = vector[0].magnitude();
            float v1_magnitude = vector[1].magnitude();
            if (v0_magnitude != 0.0 && v1_magnitude != 0.0)
            {
                vector[0].normalize();
                vector[1].normalize();

                float sign_f = sign ? -1f : 1f;
                double x = (double)vector[0].cross(vector[1]).magnitude();
                double y = (double)vector[0].dot(vector[1]);

                float theta = (float)Math.Atan2(sign_f * x, sign_f * y);

                return theta;// -theta;
            }
            else
            {
                return 0.0f;
            }
        }

        public static float getAngleXY(SkeletonPoint a, SkeletonPoint b, SkeletonPoint c)
        {
            Vecto3Float[] vector = new Vecto3Float[2];
            vector[0] = new Vecto3Float(c.X - b.X, c.Y - b.Y, 0);
            vector[1] = new Vecto3Float(a.X - b.X, a.Y - b.Y, 0);


            float v0_magnitude = vector[0].magnitude();
            float v1_magnitude = vector[1].magnitude();
            if (v0_magnitude != 0.0 && v1_magnitude != 0.0)
            {
                vector[0].normalize();
                vector[1].normalize();

                double x = (double)vector[0].cross(vector[1]).magnitude();
                double y = (double)vector[0].dot(vector[1]);

                float theta = (float)Math.Atan2(x, y);

                return -theta;
            }
            else
            {
                return 0.0f;
            }
        }

        public static float getAngleZX(SkeletonPoint a, SkeletonPoint b, SkeletonPoint c)
        {
            Vecto3Float[] vector = new Vecto3Float[2];
            vector[0] = new Vecto3Float(c.X - b.X, 0, c.Z - b.Z);
            vector[1] = new Vecto3Float(a.X - b.X, 0, a.Z - b.Z);


            float v0_magnitude = vector[0].magnitude();
            float v1_magnitude = vector[1].magnitude();
            if (v0_magnitude != 0.0 && v1_magnitude != 0.0)
            {
                vector[0].normalize();
                vector[1].normalize();

                double x = (double)vector[0].cross(vector[1]).magnitude();
                double y = (double)vector[0].dot(vector[1]);

                float theta = (float)Math.Atan2(x, y);

                return -theta;
            }
            else
            {
                return 0.0f;
            }
        }
        public static float getAngleZY(SkeletonPoint a, SkeletonPoint b, SkeletonPoint c)
        {
            Vecto3Float[] vector = new Vecto3Float[2];
            //vector[0] = new Vecto3Float(0, b.Y - c.Y, b.Z - c.Z);
            vector[0] = new Vecto3Float(0, c.Y - b.Y, c.Z - b.Z);
            vector[1] = new Vecto3Float(0, a.Y - b.Y, a.Z - b.Z);


            float v0_magnitude = vector[0].magnitude();
            float v1_magnitude = vector[1].magnitude();
            if (v0_magnitude != 0.0 && v1_magnitude != 0.0)
            {
                vector[0].normalize();
                vector[1].normalize();

                double x = (double)vector[0].cross(vector[1]).magnitude();
                double y = (double)vector[0].dot(vector[1]);

                float theta = (float)Math.Atan2(x, y);
                float theta2 = (float)Math.Atan2(-x, y);
                float theta3 = (float)Math.Atan2(x, -y);
                float theta4 = (float)Math.Atan2(-x, -y);

                return -theta;
            }
            else
            {
                return 0.0f;
            }
        }

    }
}


