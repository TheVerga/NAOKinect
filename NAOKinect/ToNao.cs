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
        private RobotPostureProxy postureproxy;
        private MemoryProxy memProxy;
        private float fractionSpeed = 0.1f;
        private Dictionary<JointType,SkeletonPoint> calibrationValue;
        private Dictionary< String, float> jointAngles;
        private Kinect sensor;
        private int avg;
        private int iteration;
        private int avgCalibration;
        private int iterationCalbration;


        public ToNao(String IP, int port, Kinect kinect, int avg)
        {
            this.motioProxy = new MotionProxy(IP, port);
            this.memProxy = new MemoryProxy(IP, port);
            this.postureproxy = new RobotPostureProxy(IP, port);
            this.sensor = kinect;
            this.calibrationValue = new Dictionary<JointType, SkeletonPoint>();
            this.jointAngles = new Dictionary<string, float>();
            this.avg = avg;
            this.avgCalibration = 5;
            this.iterationCalbration = 0;

            iteration = 0;
            sensor.SkeletonReady += calibrate;
            setUpjointAngleDictionary();
        }


        private void setUpjointAngleDictionary(){
            foreach(String x in NAOConversion.listOfTheJoint()){
                this.jointAngles.Add(x,0f);
            }
        }

        public void setUpNao()
        {
            motioProxy.setSmartStiffnessEnabled(true);
            motioProxy.wakeUp();
            postureproxy.goToPosture("Stand", 0.5f);//Stand LyingBack
        }

        public void calibrate(Skeleton[] skeletons)
        {
            Skeleton skel = (from trackskeleton in skeletons
                             where trackskeleton.TrackingState == SkeletonTrackingState.Tracked
                             select trackskeleton).FirstOrDefault();
            if (skel != null)
            {
                if (this.iterationCalbration == 0)
                {
                    foreach (Joint joint in skel.Joints)
                    {
                        //Calibration is made getting the first position of the skeleton
                        //the position of the user should be almost equal to the init pose of the NAO
                        this.calibrationValue.Add(joint.JointType, JointUtilities.averagePoint(joint.Position, this.avgCalibration));
                    }
                    this.iterationCalbration++;
                }
                else if(this.iterationCalbration > 0 && this.iterationCalbration < this.avgCalibration)
                {

                    foreach (Joint joint in skel.Joints)
                    {
                        this.calibrationValue[joint.JointType] = JointUtilities.averagePoint(this.calibrationValue[joint.JointType],
                                                                 joint.Position,
                                                                 this.avgCalibration);
                    }
                    this.iterationCalbration++;
                }
                else {
                    this.endCalibration();
                }   
            }

            

        }

        private void endCalibration()
        {
            this.sensor.SkeletonReady -= calibrate;
            this.sensor.SkeletonReady += convertSkeleton;
        }


        ///<summary>
        /// Receive a vector of skeleton from which get the first or the default one.
        /// Using the joint position compute the joint angles, store them and when it is
        /// OK pass those value to NAO
        ///</summary> 
        public void convertSkeleton(Skeleton[] skeletons)
        {
             Skeleton skel = (from trackskeleton in skeletons
                             where trackskeleton.TrackingState == SkeletonTrackingState.Tracked
                             select trackskeleton).FirstOrDefault();
            if (skel != null)
            {
                this.iteration++;
                this.jointMovement(skel);
                if (iteration == avg)
                {
                    this.moveJoint();   
                    this.iteration = 0;
                }
            }
        }


        private void jointMovement(Skeleton skel)
        {
            
            //ELBOWROLL
            //Left
            SkeletonPoint elbowLeftPos = skel.Joints[JointType.ElbowLeft].Position;
            SkeletonPoint wristLeftPos = skel.Joints[JointType.WristLeft].Position;
            SkeletonPoint shoulderLeftPos = skel.Joints[JointType.ShoulderLeft].Position;
            float elbowLeftRollAngle = Kinematic.getAngle(shoulderLeftPos, wristLeftPos, elbowLeftPos, false);
            this.jointAngles[NAOConversion.LElbowRoll] = this.jointAngles[NAOConversion.LElbowRoll]
                                                        + NAOConversion.convertAngle(NAOConversion.LElbowRoll,elbowLeftRollAngle) / this.avg;
            //Right
            SkeletonPoint elbowRightPos = skel.Joints[JointType.ElbowRight].Position;
            SkeletonPoint shoulderRightPos = skel.Joints[JointType.ShoulderRight].Position;
            SkeletonPoint wristRightPos = skel.Joints[JointType.WristRight].Position;
            float elbowRighttRollAngle = Kinematic.getAngle(shoulderRightPos,wristRightPos ,elbowRightPos , false);
            this.jointAngles[NAOConversion.RElbowRoll] = this.jointAngles[NAOConversion.RElbowRoll]
                                                         + NAOConversion.convertAngle(NAOConversion.RElbowRoll,elbowRighttRollAngle) / this.avg;

            //SHOULDERROLL
            SkeletonPoint shoulderCenterPos = skel.Joints[JointType.ShoulderCenter].Position;

            //Left 
            SkeletonPoint hipLeftpos = skel.Joints[JointType.HipLeft].Position;                
            float shoulderLeftRollAngle = Kinematic.getAngleZX(hipLeftpos, shoulderLeftPos,
                                                                elbowLeftPos);
            this.jointAngles[NAOConversion.LShoulderRoll] = this.jointAngles[NAOConversion.LShoulderRoll]
                                                            + NAOConversion.convertAngle(NAOConversion.LShoulderRoll,shoulderLeftRollAngle) / this.avg;
            //Right
            SkeletonPoint hipRightpos = skel.Joints[JointType.HipRight].Position;
            float shoulderRightRollAngle = Kinematic.getAngleZX(hipRightpos, shoulderRightPos,
                                                            elbowRightPos);
            this.jointAngles[NAOConversion.RShoulderRoll] = this.jointAngles[NAOConversion.RShoulderRoll]
                                                            + NAOConversion.convertAngle(NAOConversion.RShoulderRoll,shoulderRightRollAngle) / this.avg;

            //SHOULDERPITCH
            //TODO- Invece di settarlo al massimo si potrebbe cercare di calcolare l' angolo una volta che il braccio passa dietro la schiena
                    //Valori trovati passano da 0.81 a 0.55, quindi da 46° a 31° circa
                    //la prima idea potrebbe essere pi/2 + angolo calcoato
            //Left

            float shoulderLeftPitchAngle = Kinematic.getAngleZY(hipLeftpos, shoulderLeftPos,
                                                               elbowLeftPos); ;

            //if (elbowLeftPos.Z < this.calibrationValue[JointType.ElbowLeft].Z)
            //{
            //    shoulderLeftPitchAngle = Kinematic.getAngleZY(hipLeftpos, shoulderLeftPos,
            //                                                   elbowLeftPos);
            //}
            //else
            //{
            //    shoulderLeftPitchAngle = (float)Math.PI;
            //}

            float conv = NAOConversion.convertAngle(NAOConversion.LShoulderPitch, shoulderLeftPitchAngle);
            float avgd = conv / this.avg;
            this.jointAngles[NAOConversion.LShoulderPitch] = this.jointAngles[NAOConversion.LShoulderPitch]
                                                                + avgd;
            
            //Right
            float shoulderRightPitchAngle;

            if (elbowRightPos.Z < this.calibrationValue[JointType.ElbowRight].Z)
            {
               shoulderRightPitchAngle = Kinematic.getAngleZY(hipRightpos, shoulderRightPos,
                                                                elbowRightPos);
            }
            else
            {
                shoulderRightPitchAngle = (float)Math.PI;
            }
         
            this.jointAngles[NAOConversion.RShoulderPitch] = this.jointAngles[NAOConversion.RShoulderPitch]
                                                               + NAOConversion.convertAngle(NAOConversion.RShoulderPitch,shoulderRightPitchAngle) / this.avg;




            //Movemet with rotation matrix
            //ELBOWYAW
            //Left
            Matrix3x3 m_l = new Matrix3x3(skel.BoneOrientations[JointType.ElbowLeft].HierarchicalRotation.Matrix);
            Vecto3Float v_l = Kinematic.computeEulerFromMatrixXYZ(m_l); 
            this.jointAngles[NAOConversion.LElbowYaw] = this.jointAngles[NAOConversion.LElbowYaw]
                                                                + NAOConversion.convertAngle(NAOConversion.LElbowYaw,v_l.Z) / this.avg;
            //Right
            Matrix3x3 m_r = new Matrix3x3(skel.BoneOrientations[JointType.ElbowRight].HierarchicalRotation.Matrix);
            Vecto3Float v_r = Kinematic.computeEulerFromMatrixXYZ(m_r);
            this.jointAngles[NAOConversion.RElbowYaw] = this.jointAngles[NAOConversion.RElbowYaw]
                                                                + NAOConversion.convertAngle(NAOConversion.RElbowYaw,v_r.Z) / this.avg;

            //HEAD
            Matrix3x3 m_head = new Matrix3x3(skel.BoneOrientations[JointType.Head].HierarchicalRotation.Matrix);
            Vecto3Float v_head = Kinematic.computeEulerFromMatrixXYZ(m_head);
            //Yaw
            this.jointAngles[NAOConversion.HeadYaw] = this.jointAngles[NAOConversion.HeadYaw]
                                                                + NAOConversion.convertAngle(NAOConversion.HeadYaw,v_head.Z) / this.avg;
            //Pitch
            this.jointAngles[NAOConversion.HeadPitch] = this.jointAngles[NAOConversion.HeadPitch]
                                                               + NAOConversion.convertAngle(NAOConversion.HeadPitch,v_head.X) / this.avg;


        }

        private  void moveJoint()
        {
            //Activate whole body balance control
            //Fixing both left and right legs on a plane, 
            //they can move only along X and Y axis and rotate aroun Z axis
            //Then set the balance constraint 
            motioProxy.wbEnable(true);
            motioProxy.wbFootState("Fixed", "RLeg");
            motioProxy.wbFootState("Fixed", "LLeg");
            motioProxy.wbEnableBalanceConstraint(true, "Legs");

            System.Collections.ArrayList angles = new System.Collections.ArrayList();
            foreach (String x in NAOConversion.listOfTheJoint())
            {

                if ( x == NAOConversion.LElbowRoll || x == NAOConversion.LShoulderPitch || x == NAOConversion.LShoulderRoll || 
                     x == NAOConversion.LElbowYaw)                   
                {
                    motioProxy.setAngles(x, this.jointAngles[x], fractionSpeed);
                    System.Console.WriteLine(this.jointAngles[x]);
                }
                      
                this.jointAngles[x] = 0f;                
            }
            
            //motioProxy.angleInterpolationWithSpeed(l,angles,fractionSpeed);
        }

    }
}
