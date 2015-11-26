using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace NAOKinect
{
    class JointUtilities
    {

        public static SkeletonPoint resetPoint()
        {
            SkeletonPoint p = new SkeletonPoint();
            p.X = 0;
            p.Y = 0;
            p.Z = 0;

            return p;
        }
        public static SkeletonPoint averagePoint(SkeletonPoint p1, SkeletonPoint p2, int avgFactor)
        {
            p1.X = p1.X + p2.X / avgFactor;
            p1.Y = p1.Y + p2.Y / avgFactor; 
            p1.Z = p1.Z + p2.Z / avgFactor;
            return p1;
        }

        public static SkeletonPoint averagePoint(SkeletonPoint p1, int avgFactor)
        {
            p1.X = p1.X  / avgFactor;
            p1.Y = p1.Y  / avgFactor;
            p1.Z = p1.Z  / avgFactor;
            return p1;
        }




        ///<summary>
        ///Compare 2 skeleton point and return true if they are similar else retun false
        ///</summary> 
        public static bool comparePoint(SkeletonPoint p1, SkeletonPoint p2)
        {
            float z = Math.Abs((p1.Z) - (p2.Z));
            float y = Math.Abs((p1.Y) - (p2.Y));
            float x = Math.Abs((p1.X) - (p2.X));
            return z > 0.1f ||
                    y > 0.1f ||
                     x > 0.1;
        }

 
    }
}
