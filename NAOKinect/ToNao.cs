using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Aldebaran.Proxies;
using Microsoft.Kinect;

namespace NAOKinect
{
    public delegate void AnglesSentHandler(Dictionary<string, float> angles);
    class ToNao
    {
        private MotionProxy motioProxy;
        private RobotPostureProxy postureproxy;
        private MemoryProxy memProxy;
        private float fractionSpeed = 0.1f;
        private Dictionary<JointType,SkeletonPoint> calibrationValue;
        private Dictionary< String, float> jointAngles;
        private Dictionary<JointType, SkeletonPoint> avgPoint;
        private Kinect sensor;
        private int avg;
        private int iteration;
        private int avgCalibration;
        private int iterationCalbration;

        public event AnglesSentHandler AnglesSent;

        public ToNao(String IP, int port, Kinect kinect, int avg)
        {
            this.motioProxy = new MotionProxy(IP, port);
            this.memProxy = new MemoryProxy(IP, port);
            this.postureproxy = new RobotPostureProxy(IP, port);
            this.sensor = kinect;
            this.calibrationValue = new Dictionary<JointType, SkeletonPoint>();
            this.jointAngles = new Dictionary<string, float>();
            this.avgPoint = new Dictionary<JointType, SkeletonPoint>();
            this.avg = avg;
            this.iteration = 0;
            this.avgCalibration = 5;
            this.iterationCalbration = 0;

            setUpjointAngleDictionary();
            sensor.SkeletonReady += calibrate;
        }

        protected virtual void OnAnglesSent(EventArgs e)
        {
            AnglesSentHandler handler = AnglesSent;
            if (handler != null)
            {
                handler(jointAngles);
            }
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


        ///<summary>
        /// At the beginnig a calibration is made by making an average of 5 frame
        ///</summary>
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
            this.sensor.SkeletonReady += getPoint;
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
                        this.avgPoint.Add(joint.JointType, JointUtilities.averagePoint(joint.Position, this.avgCalibration));
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
                                                                                     this.avgCalibration);
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
        private void angleMatrix(Skeleton skel){
            //ELBOWYAW
            //Left
            Matrix3x3 m_l = new Matrix3x3(skel.BoneOrientations[JointType.ElbowLeft].HierarchicalRotation.Matrix);
            Vecto3Float v_l = Kinematic.computeEulerFromMatrixXYZ(m_l);
            this.jointAngles[NAOConversion.LElbowYaw] = this.jointAngles[NAOConversion.LElbowYaw]
                                                                + NAOConversion.convertAngle(NAOConversion.LElbowYaw, v_l.Z) / this.avg;
            //Right
            Matrix3x3 m_r = new Matrix3x3(skel.BoneOrientations[JointType.ElbowRight].HierarchicalRotation.Matrix);
            Vecto3Float v_r = Kinematic.computeEulerFromMatrixXYZ(m_r);
            this.jointAngles[NAOConversion.RElbowYaw] = this.jointAngles[NAOConversion.RElbowYaw]
                                                                + NAOConversion.convertAngle(NAOConversion.RElbowYaw, v_r.Z) / this.avg;

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
            
            //ELBOWROLL
            //Left
            float elbowLeftRollAngle = Kinematic.getAngle(this.avgPoint[JointType.ShoulderLeft], 
                                                          this.avgPoint[JointType.WristLeft], 
                                                          this.avgPoint[JointType.ElbowLeft],
                                                          false);

            
            this.jointAngles[NAOConversion.LElbowRoll] = NAOConversion.convertAngle(NAOConversion.LElbowRoll, elbowLeftRollAngle );
            //Right
            float elbowRighttRollAngle = Kinematic.getAngle(this.avgPoint[JointType.ShoulderRight],
                                                            this.avgPoint[JointType.WristRight],
                                                            this.avgPoint[JointType.ElbowRight],
                                                            false);

            this.jointAngles[NAOConversion.RElbowRoll] = NAOConversion.convertAngle(NAOConversion.RElbowRoll, elbowRighttRollAngle );

            //SHOULDERROLL
            //Left 
            float shoulderLeftRollAngle = Kinematic.getAngleZX(this.avgPoint[JointType.HipLeft],
                                                               this.avgPoint[JointType.ShoulderLeft],
                                                               this.avgPoint[JointType.ElbowLeft]);
            this.jointAngles[NAOConversion.LShoulderRoll] = NAOConversion.convertAngle(NAOConversion.LShoulderRoll, shoulderLeftRollAngle );
            //Right
            float shoulderRightRollAngle = Kinematic.getAngleZX( this.avgPoint[JointType.ElbowRight],
                                                                 this.avgPoint[JointType.ShoulderRight],
                                                                 this.avgPoint[JointType.HipRight]);
            this.jointAngles[NAOConversion.RShoulderRoll] = NAOConversion.convertAngle(NAOConversion.RShoulderRoll, shoulderRightRollAngle );

            //SHOULDERPITCH
            //Left
            if (this.avgPoint[JointType.ElbowLeft].Z < this.avgPoint[JointType.Spine].Z ||
                this.avgPoint[JointType.ElbowLeft].Y >= this.avgPoint[JointType.Spine].Y)
            {
                float shoulderLeftPitchAngle = Kinematic.getAngleZY(this.avgPoint[JointType.ElbowLeft],
                                                                this.avgPoint[JointType.ShoulderLeft],
                                                                this.avgPoint[JointType.HipLeft]);
                this.jointAngles[NAOConversion.LShoulderPitch] = NAOConversion.convertAngle(NAOConversion.LShoulderPitch, shoulderLeftPitchAngle);
            }
            else
            {
                this.jointAngles[NAOConversion.LShoulderPitch] = (float)Math.PI/2;
            }


            
            //Right
            if (this.avgPoint[JointType.ElbowRight].Z < this.avgPoint[JointType.Spine].Z ||
                 this.avgPoint[JointType.ElbowRight].Y >= this.avgPoint[JointType.Spine].Y)
            {
                float shoulderRightPitchAngle = Kinematic.getAngleZY(this.avgPoint[JointType.HipRight],
                                                               this.avgPoint[JointType.ShoulderRight],
                                                               this.avgPoint[JointType.ElbowRight]);
                this.jointAngles[NAOConversion.RShoulderPitch] = NAOConversion.convertAngle(NAOConversion.RShoulderPitch, shoulderRightPitchAngle);
            }
            else
            {
                this.jointAngles[NAOConversion.RShoulderPitch] = (float)Math.PI/2;
            }
         
        }

        ///<summary>
        /// Send to the NAO the angle just computed.
        ///</summary>
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

            System.Collections.ArrayList joint = new System.Collections.ArrayList();
            foreach (String x in NAOConversion.listOfTheJoint())
            {
                    motioProxy.setAngles(x, this.jointAngles[x], fractionSpeed);
                    AnglesSent(jointAngles);
                    //System.Console.WriteLine(this.jointAngles[x]);                      
                    this.jointAngles[x] = 0f;                
            }
            //motioProxy.angleInterpolationWithSpeed(joint, angles, fractionSpeed);
        }

        public Dictionary<String, float> getAllJointAngles()
        {
            return jointAngles;
        }
        public float getJointAngles(string x)
        {
            return jointAngles[x];
        }


    }
}
