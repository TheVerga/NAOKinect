using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Aldebaran.Proxies;
using Microsoft.Kinect;


namespace NAOKinect
{
    class ToNao
    {
        private MotionProxy motioProxy;
        private Aldebaran.Proxies.MemoryProxy memProxy;
        private float fractionSpeed = 0.5f;


        public ToNao(String IP, int port, Kinect kinect)
        {
            this.motioProxy = new MotionProxy(IP, port);
            this.memProxy = new MemoryProxy(IP, port);
            kinect.SkeletonReady += move;
        }

        public void setUpNao()
        {
             motioProxy.wakeUp();
            //maybe some other thing
        }

        public void move(Skeleton[] skeletons)
        {
            //getting a tracked skeleton if any, else return a default one
            Skeleton skel = (from trackskeleton in skeletons
                             where trackskeleton.TrackingState == SkeletonTrackingState.Tracked
                             select trackskeleton).FirstOrDefault();
            if (skel != null)
            {
                //iterate on the skeleton found and "try" to move the NAO
                foreach (BoneOrientation orientation in skel.BoneOrientations)
                {
                    //Vecto3Float eulerM1 = Kinematic.computeEulerFromMatrixZYX(orientation.HierarchicalRotation.Matrix);
                    //Vecto3Float eulerQ = Kinematic.computeEulerFromQuaternion(orientation.HierarchicalRotation.Quaternion);
                    //Vecto3Float eulerM2 = Kinematic.computeEulerFromMatrixXYZ(orientation.HierarchicalRotation.Matrix);
                    Vecto3Float eulerM3 = Kinematic.computeEulerFromMatrixZXY(orientation.HierarchicalRotation.Matrix);
                    this.setAngleFromOrientation(eulerM3, orientation.StartJoint,orientation.EndJoint);
                }
            }
        }

        public static float convertToRadians(int angle)
        {
            return (float) (angle* (Math.PI / 180));
        }


        private void setAngleFromOrientation(Vecto3Float rotation, JointType startJoint,JointType endJoint)
        {

            motioProxy.setStiffnesses("Body", 1.0f);
            motioProxy.setSmartStiffnessEnabled(true);
            if (startJoint == JointType.ShoulderRight)
            {
                
                //motioProxy.setAngles(NAOJointName.RShoulderPitch, 0f fractionSpeed);
                //motioProxy.setAngles(NAOJointName.RShouderRoll, 0f, fractionSpeed);
            }
            else if (startJoint == JointType.ShoulderLeft && endJoint == JointType.ElbowLeft)
            {

                motioProxy.setAngles(NAOJointName.LShoulderPitch, rotation.Y , fractionSpeed);
                motioProxy.setAngles(NAOJointName.LShoulderRoll, rotation.X + 0.28f, fractionSpeed);
                motioProxy.setAngles(NAOJointName.LElbowYaw, -rotation.Z , fractionSpeed);
                
            }
            else if (startJoint == JointType.ElbowLeft && endJoint == JointType.WristLeft)
            {
                motioProxy.setAngles(NAOJointName.LElbowRoll, rotation.X , fractionSpeed);
                motioProxy.setAngles(NAOJointName.LElbowYaw, -rotation.Z , fractionSpeed);
            }
            else if (startJoint == JointType.WristLeft)
            {
                //motioProxy.setAngles(NAOJointName.LWristYaw, rotation.Z, fractionSpeed);
            }
            else if (startJoint == JointType.ElbowRight && endJoint == JointType.WristRight)
            {
                //motioProxy.setAngles(NAOJointName.RElbowRoll, convertToRadians(-80), fractionSpeed);
                //motioProxy.setAngles(NAOJointName.RElbowYaw, rotation.Z, fractionSpeed);
            }
            else if (startJoint == JointType.WristRight)
            {
                //motioProxy.setAngles(NAOJointName.RWristYaw, 0f, fractionSpeed);
            }
            else if (endJoint == JointType.Head)
            {
                motioProxy.setAngles(NAOJointName.HeadPitch, -rotation.X, fractionSpeed);
                motioProxy.setAngles(NAOJointName.HeadYaw, rotation.Y, fractionSpeed);
            }
        }
    }

    class NAOJointName
    {

        public static String HeadPitch = "HeadPitch";
        public static String HeadYaw = "HeadYaw";


        public static String LShoulderRoll = "LShoulderRoll";
        public static String LShoulderPitch = "LShoulderPitch";


        public static String RShouderRoll = "RShoulderRoll";
        public static String RShoulderPitch = "RShoulderPitch";

        public static String LElbowYaw = "LElbowYaw";
        public static String LElbowRoll = "LElbowRoll";


        public static String RElbowYaw = "RElbowYaw";
        public static String RElbowRoll = "RElbowRoll";


        public static String LWristYaw = "LWristYaw";


        public static String RWristYaw = "RWristYaw";


        public static String LHipYaw = "LHipYaw";
        public static String LHipPitch = "LHipPitch";
        public static String LHipRoll = "LHipRoll";


        public static String RHipYaw = "RHipYaw";
        public static String RHipPitch = "RHipPitch";
        public static String RHipRoll = "RHipRoll";


        public static String LKneePitch = "LKneePitch";


        public static String RKneePitch = "RKneePitch";


        public static String LAnklePitch = "LAnklePitch";
        public static String LAnkleRoll = "LAnkleRoll";


        public static String RAnklePitch = "RAnklePitch";
        public static String RAnkleRoll = "RAnkleRoll";
    }
}
