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
        public static Vecto3Float computeEulerFromMatrix(Matrix4 m)
        {
            Vecto3Float sol = new Vecto3Float();
            if (m.M13 < 1f)
            {
                if (m.M13 > -1f)
                {
                    sol.X = (float)Math.Atan2((double)-m.M23, (double)m.M33);
                    sol.Y = (float)Math.Asin((double)m.M13);
                    sol.Z = (float)Math.Atan2((double)-m.M12, (double)m.M11);
                }
                else
                {
                    sol.X = -(float)Math.Atan2((double)-m.M21, (double)m.M22);
                    sol.Y = -(float)(Math.PI) / 2;
                    sol.Z = 0f;
                }

            }
            else
            {
                sol.X = (float)Math.Atan2((double)-m.M21, (double)m.M22);
                sol.Y = (float)(Math.PI) / 2;
                sol.Z = 0f;
            }
            return sol;
        }

        ///<summary>
        /// Receive a rotation matrix of type Vector4
        /// Return a Vector3Float containing the rotation around the 3 axis
        ///</summary>
        public static Vecto3Float computeEulerFromQuaternion(Vector4 q)
        {

            // Store the Euler angles in radians
            Vecto3Float pitchYawRoll = new Vecto3Float();

            double sqw = q.W * q.W;
            double sqx = q.X * q.X;
            double sqy = q.Y * q.Y;
            double sqz = q.Z * q.Z;

            // If quaternion is normalised the unit is one, otherwise it is the correction factor
            double unit = sqx + sqy + sqz + sqw;
            double test = q.X * q.Y + q.Z * q.W;

            if (test > 0.4999f * unit)                              // 0.4999f OR 0.5f - EPSILON
            {
                // Singularity at north pole
                pitchYawRoll.Y = 2f * (float)Math.Atan2(q.X, q.W);  // Yaw
                pitchYawRoll.X = (float)Math.PI * 0.5f;             // Pitch
                pitchYawRoll.Z = 0f;                                // Roll
                return pitchYawRoll;
            }
            else if (test < -0.4999f * unit)                        // -0.4999f OR -0.5f + EPSILON
            {
                // Singularity at south pole
                pitchYawRoll.Y = -2f * (float)Math.Atan2(q.X, q.W); // Yaw
                pitchYawRoll.X = (float)-Math.PI * 0.5f;            // Pitch
                pitchYawRoll.Z = 0f;                                // Roll
                return pitchYawRoll;
            }
            else
            {
                pitchYawRoll.Y = (float)Math.Atan2(2f * q.Y * q.W - 2f * q.X * q.Z, sqx - sqy - sqz + sqw);       // Yaw
                pitchYawRoll.X = (float)Math.Asin(2f * test / unit);                                             // Pitch
                pitchYawRoll.Z = (float)Math.Atan2(2f * q.X * q.W - 2f * q.Y * q.Z, -sqx + sqy - sqz + sqw);      // Roll
            }

            return pitchYawRoll;
        }
    }
}
