using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Aldebaran.Proxies;
using Microsoft.Kinect;

namespace NAOKinect
{
    class ToNao2
    {
        private MotionProxy motioProxy;
        private RobotPostureProxy postureproxy;
        private MemoryProxy memProxy;
        private float fractionSpeed = 0.05f;
        private Dictionary<String, float> jointAngles;
        private Dictionary<JointType, SkeletonPoint> avgPoint;
        private Kinect sensor;
        private int avg;
        private int iteration;


        public ToNao2(String IP, int port, Kinect kinect, int avg)
        {
            this.motioProxy = new MotionProxy(IP, port);
            this.memProxy = new MemoryProxy(IP, port);
            this.postureproxy = new RobotPostureProxy(IP, port);
            this.sensor = kinect;
            this.jointAngles = new Dictionary<string, float>();
            this.avgPoint = new Dictionary<JointType, SkeletonPoint>();
            this.avg = avg;
            this.iteration = 0;

            setUpjointAngleDictionary();
        }

        private void setUpjointAngleDictionary()
        {
            foreach (String x in NAOConversion.listOfTheJoint())
            {
                this.jointAngles.Add(x, 0f);
            }
        }

        public void setUpNao()
        {
            motioProxy.setSmartStiffnessEnabled(true);
            motioProxy.wakeUp();
            postureproxy.goToPosture("Stand", 0.5f);//Stand LyingBack            
            this.sensor.SkeletonReady += getPoint;
        }

        public void setDownNao()
        {
            if (sensor != null)
            {
                this.sensor.SkeletonReady -= getPoint;
            }
            motioProxy.rest();
        }


        ///<summary>
        /// 
        ///</summary>
        public void getPoint(Skeleton[] skeletons)
        {
            Skeleton skel = (from trackskeleton in skeletons
                             where trackskeleton.TrackingState == SkeletonTrackingState.Tracked
                             select trackskeleton).FirstOrDefault();
            if (skel != null)
            {
                if (this.iteration == 0)
                {
                    //fill the dictionary with the new point
                    foreach (Joint joint in skel.Joints)
                    {
                        this.avgPoint.Add(joint.JointType, JointUtilities.averagePoint(joint.Position, this.avg));
                    }
                    this.angleMatrix(skel);
                    this.iteration++;
                }
                else if (this.iteration > 0 && this.iteration < this.avg)
                {
                    //Continue to fill the dictionary making an average with the old inserted point
                    foreach (Joint joint in skel.Joints)
                    {
                        this.avgPoint[joint.JointType] = JointUtilities.averagePoint(this.avgPoint[joint.JointType],
                                                                                     joint.Position,
                                                                                     this.avg);
                    }
                    this.angleMatrix(skel);
                    this.iteration++;
                }
                else
                {
                    //call the function to compute the angle
                    this.angleVector();
                    //call the function to move the joint
                    this.moveJoint();
                    //reset iteration
                    this.iteration = 0;
                    //remove the old value from the dictionary
                    this.avgPoint.Clear();

                }
            }
        }


        ///<summary>
        /// Compute some angle using rotation matrix
        ///</summary>
        private void angleMatrix(Skeleton skel)
        {
            //HEAD
            Matrix3x3 m_head = new Matrix3x3(skel.BoneOrientations[JointType.Head].HierarchicalRotation.Matrix);
            Vecto3Float v_head = Kinematic.computeEulerFromMatrixXYZ(m_head);
            //Yaw
            this.jointAngles[NAOConversion.HeadYaw] = this.jointAngles[NAOConversion.HeadYaw]
                                                                + NAOConversion.convertAngle(NAOConversion.HeadYaw, v_head.Z) / this.avg;
            //Pitch
            this.jointAngles[NAOConversion.HeadPitch] = this.jointAngles[NAOConversion.HeadPitch]
                                                               + NAOConversion.convertAngle(NAOConversion.HeadPitch, v_head.X) / this.avg;
        }

        ///<summary>
        /// Compute angle between the joint according to the position of the joint
        ///</summary>
        private void angleVector()
        {
            SkeletonPoint shoulderLeft = this.avgPoint[JointType.ShoulderLeft];
            SkeletonPoint shoulderCenter = this.avgPoint[JointType.ShoulderCenter];
            SkeletonPoint shoulderRight = this.avgPoint[JointType.ShoulderRight];

            SkeletonPoint elbowLeft = this.avgPoint[JointType.ElbowLeft];
            SkeletonPoint elbowRight = this.avgPoint[JointType.ElbowRight];

            SkeletonPoint wristLeft = this.avgPoint[JointType.WristLeft];
            SkeletonPoint wristRight = this.avgPoint[JointType.WristRight];

            SkeletonPoint hipLeft = this.avgPoint[JointType.HipLeft];            
            SkeletonPoint hipCenter = this.avgPoint[JointType.HipCenter];
            SkeletonPoint hipRight = this.avgPoint[JointType.HipRight];

            Vecto3Float upperArmLeft = new Vecto3Float(shoulderLeft.X - elbowLeft.X,
                                                       shoulderLeft.Y - elbowLeft.Y,
                                                       shoulderLeft.Z - elbowLeft.Z);

            Vecto3Float lowerArmLeft = new Vecto3Float(wristLeft.X - elbowLeft.X,
                                                       wristLeft.Y - elbowLeft.Y,
                                                       wristLeft.Z - elbowLeft.Z);

            Vecto3Float upperArmRight = new Vecto3Float(shoulderRight.X - elbowRight.X,
                                                        shoulderRight.Y - elbowRight.Y,
                                                        shoulderRight.Z - elbowRight.Z);

            Vecto3Float lowerArmRight = new Vecto3Float(wristRight.X - elbowRight.X,
                                                        wristRight.Y - elbowRight.Y,
                                                        wristRight.Z - elbowRight.Z);


            Vecto3Float shoulderLToHipLeft = new Vecto3Float(shoulderLeft.X - hipLeft.X,
                                                             shoulderLeft.Y - hipLeft.Y,
                                                             shoulderLeft.Z - hipLeft.Z);

            Vecto3Float shoulderRToHipRight = new Vecto3Float(shoulderRight.X - hipRight.X,
                                                              shoulderRight.Y - hipRight.Y,
                                                              shoulderRight.Z - hipRight.Z);

            Vecto3Float shoulderLToShoulderC = new Vecto3Float(shoulderLeft.X - shoulderCenter.X,
                                                               shoulderLeft.Y - shoulderCenter.Y,
                                                               shoulderLeft.Z - shoulderCenter.Z);

            Vecto3Float shoulderRToShoulderC = new Vecto3Float(shoulderRight.X - shoulderCenter.X,
                                                               shoulderRight.Y - shoulderCenter.Y,
                                                               shoulderRight.Z - shoulderCenter.Z);


            if (hipLeft.Z + 0.15 > elbowLeft.Z)   //if the elbow is in front of the hip                              
            {
                //Shoulder Roll
                //Left

                if (Math.Abs(shoulderLeft.Z - elbowLeft.Z) < 0.1 &&
                    Math.Abs(shoulderLeft.X - elbowLeft.X) < 0.1)
                {
                    this.jointAngles[NAOConversion.LShoulderRoll] = NAOConversion.convertAngle(NAOConversion.LShoulderRoll,
                                                                                    Kinematic.getAngleNew(upperArmLeft.projectionOntoXY(),
                                                                                                          shoulderLToHipLeft.projectionOntoXY()));
                }
                else
                {
                    this.jointAngles[NAOConversion.LShoulderRoll] = NAOConversion.convertAngle(NAOConversion.LShoulderRoll,
                                                                                     Kinematic.getAngleNew(upperArmLeft.projectionOntoZX(),
                                                                                                           shoulderLToHipLeft.projectionOntoZX()));
                }


                //Shoulder Pitch
                //Left
                this.jointAngles[NAOConversion.LShoulderPitch] = NAOConversion.convertAngle(NAOConversion.LShoulderPitch,
                                                                                             Kinematic.getAngleNew(upperArmLeft.projectionOntoZY(),
                                                                                                                   shoulderLToHipLeft.projectionOntoZY()));
            }

            if (hipRight.Z + 0.15 > elbowRight.Z)//if the elbow is in front of the hip
                
            {

                //Shoulder Roll
                //Right
                if (Math.Abs(shoulderRight.Z - elbowRight.Z) < 0.1 &&
                    Math.Abs(shoulderRight.X - elbowRight.X) < 0.1)
                {
                    this.jointAngles[NAOConversion.RShoulderRoll] = NAOConversion.convertAngle(NAOConversion.RShoulderRoll,
                                                                                    Kinematic.getAngleNew(upperArmRight.projectionOntoXY(),
                                                                                                          shoulderRToHipRight.projectionOntoXY()));
                }
                else
                {
                    this.jointAngles[NAOConversion.RShoulderRoll] = NAOConversion.convertAngle(NAOConversion.RShoulderRoll,
                                                                                     Kinematic.getAngleNew(upperArmRight.projectionOntoZX(),
                                                                                                           shoulderRToHipRight.projectionOntoZX()));
                }
                //Shoulder Pitch
                //Right
                this.jointAngles[NAOConversion.RShoulderPitch] = NAOConversion.convertAngle(NAOConversion.RShoulderPitch,
                                                                                             Kinematic.getAngleNew(shoulderRToHipRight.projectionOntoZY(),
                                                                                                                    upperArmRight.projectionOntoZY()));
            }


            //Elbow Roll
            //Left
            this.jointAngles[NAOConversion.LElbowRoll] = NAOConversion.convertAngle(NAOConversion.LElbowRoll,
                                                                                     Kinematic.getAngleNew(upperArmLeft, lowerArmLeft));
            //Right            
            this.jointAngles[NAOConversion.RElbowRoll] = NAOConversion.convertAngle(NAOConversion.RElbowRoll,
                                                                                    Kinematic.getAngleNew(upperArmRight, lowerArmRight));

            //Elbow Yaw
            ////Left  
            Vecto3Float shoulderLTohipCenter = new Vecto3Float(shoulderLeft.X - hipCenter.X,
                                                               shoulderLeft.Y - hipCenter.Y,
                                                               shoulderLeft.Z - hipCenter.Z);
            Vecto3Float lCrossCenterArm = upperArmLeft.cross(shoulderLTohipCenter);
            Vecto3Float lCrossArms = upperArmLeft.cross(lowerArmLeft);
            float lElbowYaw = Kinematic.getAngleNew(lCrossCenterArm, lCrossArms);
            this.jointAngles[NAOConversion.LElbowYaw] = NAOConversion.convertAngle(NAOConversion.LElbowYaw, lElbowYaw);


            //Right
            Vecto3Float shoulderRTohipCenter = new Vecto3Float(shoulderRight.X - hipCenter.X,
                                                               shoulderRight.Y - hipCenter.Y,
                                                               shoulderRight.Z - hipCenter.Z);
            Vecto3Float rCrossCenterArm = upperArmRight.cross(shoulderRTohipCenter);
            Vecto3Float rCrossArms = upperArmRight.cross(lowerArmRight);
            float rElbowYaw = Kinematic.getAngleNew(rCrossCenterArm, rCrossArms);
            this.jointAngles[NAOConversion.RElbowYaw] = NAOConversion.convertAngle(NAOConversion.RElbowYaw, rElbowYaw);
        }

        ///<summary>
        /// Send to the NAO the angle just computed.
        ///</summary>
        private void moveJoint()
        {
            //Activate whole body balance control
            //Fixing both left and right legs on a plane, 
            //they can move only along X and Y axis and rotate aroun Z axis
            //Then set the balance constraint 
            motioProxy.wbEnable(true);
            motioProxy.wbFootState("Fixed", "RLeg");
            motioProxy.wbFootState("Fixed", "LLeg");
            motioProxy.wbEnableBalanceConstraint(true, "Legs");

            foreach (String x in NAOConversion.listOfTheJoint())
            {
                
                    motioProxy.setAngles(x, this.jointAngles[x], fractionSpeed);
                    if (x == NAOConversion.HeadPitch || x == NAOConversion.HeadYaw)
                    {
                        this.jointAngles[x] = 0f;
                    }
                    if (x == NAOConversion.LShoulderRoll)
                    {
                        System.Console.WriteLine(this.jointAngles[x]);
                    }
            }
        }

    }
}

