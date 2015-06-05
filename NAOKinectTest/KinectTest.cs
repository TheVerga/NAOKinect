using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using NAOKinect;

namespace NAOKinectTest
{
    [TestClass]
    public class KinectTest
    {
        [TestMethod]
        public void TestMethod1()
        {
            Kinect kinect = new Kinect();
            kinect.Connect();
        }
    }
}
