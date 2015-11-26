using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Aldebaran.Proxies;
using Microsoft.Kinect;


namespace NAOKinect
{

    class NAOConversion
    {

        public static String HeadPitch = "HeadPitch";
        public static String HeadYaw = "HeadYaw";


        public static String LShoulderRoll = "LShoulderRoll";
        public static String LShoulderPitch = "LShoulderPitch";


        public static String RShoulderRoll = "RShoulderRoll";
        public static String RShoulderPitch = "RShoulderPitch";

        public static String LElbowYaw = "LElbowYaw";
        public static String LElbowRoll = "LElbowRoll";


        public static String RElbowYaw = "RElbowYaw";
        public static String RElbowRoll = "RElbowRoll";


        public static String LWristYaw = "LWristYaw";


        public static String RWristYaw = "RWristYaw";


        //LhipYawPitch and rhipYawPitch are phisically the same motor
        public static String HipYawPitch = "LHipYawPitch";

        public static String LHipPitch = "LHipPitch";
        public static String LHipRoll = "LHipRoll";
        public static String RHipPitch = "RHipPitch";
        public static String RHipRoll = "RHipRoll";


        public static String LKneePitch = "LKneePitch";


        public static String RKneePitch = "RKneePitch";


        public static String LAnklePitch = "LAnklePitch";
        public static String LAnkleRoll = "LAnkleRoll";


        public static String RAnklePitch = "RAnklePitch";
        public static String RAnkleRoll = "RAnkleRoll";

        public static float maxFootStepX = 0.080f;
        public static float minFootStepX = 0.001f;

        public static float maxFootStepY = 0.160f;
        public static float minFootStepY = 0.101f;


        public static float convertAngle(String jointName, float angle)
        {
            
            if (jointName == NAOConversion.LShoulderPitch || jointName == NAOConversion.RShoulderPitch)
            {
                angle = (float)(Math.PI/2 - angle);
                if (angle > 2.0857f)
                {
                    angle = 2.0857f;
                }
                else if (angle < -2.0857f)
                {
                    angle = -2.0857f;
                }
            }
            else if (jointName == NAOConversion.LShoulderRoll)
            {
                //-0.3142 to 1.3265
                angle = angle - (float)Math.PI / 4f;
                if (angle < -0.3142f)
                {
                    angle = -0.3142f;
                }
                else if (angle > 1.3265f)
                {
                    angle = 1.3265f;
                }
            }
            else if (jointName == NAOConversion.RShoulderRoll)
            {
                angle = -(angle - (float)Math.PI / 4f);
                if (angle > 0.3142f)
                {
                    angle = 0.3142f;
                }
                else if (angle < -1.3265f)
                {
                    angle = -1.3265f;
                }
            }
            else if (jointName == NAOConversion.LElbowYaw)
            {
                //-2.0857 to 2.0857
                angle = (angle - (float)Math.PI * 3 / 4);

                if (angle > 2.0857f)
                {
                    angle = -2.0857f;
                }
                else if (angle < -2.0857f)
                {
                    angle = -2.0857f;
                }
            }
            else if (jointName == NAOConversion.RElbowYaw)
            {
                //-2.0857 to 2.0857
                angle = -(angle - (float)Math.PI * 3 / 4);
                if (angle > 2.0857f)
                {
                    angle = -2.0857f;
                }
                else if (angle < -2.0857f)
                {
                    angle = -2.0857f;
                }
            }
            else if (jointName == NAOConversion.HeadPitch)
            {
                angle = -(angle);
            }
            else if (jointName == NAOConversion.LElbowRoll)
            {
                angle = -(float)(Math.PI - angle);
                 
                if(angle > -0.0349f){
                    angle = -0.0349f;
                }
                else if (angle < -1.5446f)
                {
                    angle = -1.5446f;
                }
            }
            else if (jointName == NAOConversion.RElbowRoll)
            {
                angle = (float)(Math.PI - angle);
                if (angle < 0.0349f)
                {
                    angle = 0.0349f;
                }
                else if (angle > 1.5446f)
                {
                    angle = 1.5446f;
                }
            }

            return angle;
        }

       
        //X must be in range [-0.0400000 <-> +0.0800000]
        public static float convertFootStepXAxis(float stepLength)
        {
            float res = stepLength / 10f;
            return res ;//> +0.08f ? +0.08f : (res < -0.04f ? -0.04f : res);
        }


        //Y must be in range [+0.0880000 <-> +0.160000]
        public static float converFootStepYAxis(float leftY,float rightY)
        {
            float stepLength = 0;
            if((leftY > 0 && rightY < 0) || (leftY < 0 && rightY > 0))
            {
                //Case in wich the user it is almost in the center
                //Actualy since the user is in front of the camera there should only
                //the case lefty < 0 && righty>0
                stepLength = Math.Abs(leftY) + Math.Abs(rightY);
            }
            else {
                stepLength = leftY > rightY ? leftY - rightY : rightY - leftY;
            }

            float res = stepLength / 10f;
            return res;// > 0.16f ? 0.16f : (res < +0.088f ? +0.088f : res);
        }

        public static System.Collections.Generic.List<string> listOfTheJoint()
        {
            System.Collections.Generic.List<string> list = new System.Collections.Generic.List<string>();
            

            list.Add("HeadPitch");
            list.Add("HeadYaw");
            list.Add("LShoulderRoll");
            list.Add("LShoulderPitch");
            list.Add("RShoulderRoll");        
            list.Add("RShoulderPitch");
            list.Add("LElbowYaw");
            list.Add("LElbowRoll");
            list.Add("RElbowYaw");
            list.Add("RElbowRoll");

            //list.Add("LKneePitch");
            //list.Add("RKneePitch");

            //list.Add("LAnklePitch");
            //list.Add("LAnkleRoll");
            //list.Add("RAnklePitch");
            //list.Add("RAnkleRoll");
            //list.Add("LWristYaw");
            //list.Add("RWristYaw");
            //list.Add("LHipYawPitch");
            //list.Add("LHipPitch");
            //list.Add("LHipRoll");
            //list.Add("RHipPitch");
            //list.Add("RHipRoll");


            return list;
        }
    }
}
