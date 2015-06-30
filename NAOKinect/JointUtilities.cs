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
        public static SkeletonPoint averagePoint(SkeletonPoint p1, SkeletonPoint p2, int avgFactor)
        {
            p1.X = p1.X + p2.X / avgFactor;
            p1.Z = p1.Z + p2.Z / avgFactor;
            return p1;
        }

        public static bool compareZXSkelPoint(SkeletonPoint p1, SkeletonPoint p2)
        {

            return p1.Z != p2.Z || p1.X != p2.X;

        }
    }
}
