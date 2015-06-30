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
        private float fractionSpeed = 0.5f;
        private Dictionary<JointType,SkeletonPoint> calibrationValue;
        private Dictionary< String, float> jointAngles;
        private Dictionary<JointType, SkeletonPoint> footPosition;
        private Kinect sensor;
        private int avg;
        private int iteration;


        public ToNao(String IP, int port, Kinect kinect, int avg)
        {
            this.motioProxy = new MotionProxy(IP, port);
            this.memProxy = new MemoryProxy(IP, port);
            this.postureproxy = new RobotPostureProxy(IP, port);
            this.sensor = kinect;
            this.calibrationValue = new Dictionary<JointType, SkeletonPoint>();
            this.jointAngles = new Dictionary<string, float>();
            this.footPosition = new Dictionary<JointType, SkeletonPoint>();
            this.avg = avg;

            iteration = 0;
            sensor.SkeletonReady += calibrate;
            setUpjointAngleDictionary();
            setUpfootPositionDictionary();
        }

        private void setUpfootPositionDictionary()
        {
            SkeletonPoint point = new SkeletonPoint();
            point.X = 0;
            point.Y = 0;
            point.Z = 0;
            this.footPosition.Add(JointType.FootLeft, point);
            this.footPosition.Add(JointType.FootRight, point);
        }

        private void setUpjointAngleDictionary(){
            foreach(String x in NAOConversion.listOfTheJoint()){
                this.jointAngles.Add(x,0f);
            }
        }

        public void setUpNao()
        {
            motioProxy.setStiffnesses("Body", 1.0f);
            motioProxy.setSmartStiffnessEnabled(true);
            motioProxy.wakeUp();
            postureproxy.goToPosture("Stand", 0.5f);
        }

        public void calibrate(Skeleton[] skeletons)
        {
            Skeleton skel = (from trackskeleton in skeletons
                             where trackskeleton.TrackingState == SkeletonTrackingState.Tracked
                             select trackskeleton).FirstOrDefault();
            if (skel != null)
            {
                foreach (Joint joint in skel.Joints)
                {  
                    //Calibration is made getting the first position of the skeleton
                    //the position of the user should be almost equal to the init pose of the NAO
                    this.calibrationValue.Add(joint.JointType, joint.Position);
                }

                this.endCalibration();
            }

            

        }

        private void endCalibration()
        {
            this.sensor.SkeletonReady -= calibrate;
            this.sensor.SkeletonReady += convertSkeleton;
        }


        ///<summary>
        /// Receive a vector of skeleton from which get the first or the default one.
        /// Using the joint position campute the angles and pass them to the NAO 
        ///</summary> 
        public void convertSkeleton(Skeleton[] skeletons)
        {
             Skeleton skel = (from trackskeleton in skeletons
                             where trackskeleton.TrackingState == SkeletonTrackingState.Tracked
                             select trackskeleton).FirstOrDefault();
            if (skel != null)
            {
                this.iteration++;
                //this.jointMovement(skel);
                this.movementFootStep(skel);
                if (iteration == avg)
                {
                    //MAybe this function should be called as a new thread
                    //and passing the dictionary jointAngle by cloning it
                    //moveJoint();   
                    this.footStepMovement(skel);
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
            float elbowLeftRollAngle = Kinematic.getAngle(shoulderLeftPos, elbowLeftPos, wristLeftPos, false);
            this.jointAngles[NAOConversion.LElbowRoll] = this.jointAngles[NAOConversion.LElbowRoll]
                                                                + NAOConversion.convertAngle(NAOConversion.LElbowRoll,elbowLeftRollAngle) / this.avg;
            //Right
            SkeletonPoint elbowRightPos = skel.Joints[JointType.ElbowRight].Position;
            SkeletonPoint shoulderRightPos = skel.Joints[JointType.ShoulderRight].Position;
            SkeletonPoint wristRightPos = skel.Joints[JointType.WristRight].Position;
            float elbowRighttRollAngle = Kinematic.getAngle(shoulderRightPos, elbowRightPos, wristRightPos, false);
            this.jointAngles[NAOConversion.RElbowRoll] = this.jointAngles[NAOConversion.RElbowRoll]
                                                                + NAOConversion.convertAngle(NAOConversion.RElbowRoll,elbowRighttRollAngle) / this.avg;

            //SHOULDERROLL
            SkeletonPoint shoulderCenterPos = skel.Joints[JointType.ShoulderCenter].Position;            
            //Left                
            float shoulderLeftRollAngle = Kinematic.getAngle(elbowLeftPos, shoulderLeftPos,
                                                           shoulderCenterPos, false);
            this.jointAngles[NAOConversion.LShoulderRoll] = this.jointAngles[NAOConversion.LShoulderRoll]
                                                                + NAOConversion.convertAngle(NAOConversion.LShoulderRoll,shoulderLeftRollAngle) / this.avg;
            //Right
            float shoulderRightRollAngle = Kinematic.getAngle(elbowRightPos, shoulderRightPos,
                                                           shoulderCenterPos, true);
            this.jointAngles[NAOConversion.RShouderRoll] = this.jointAngles[NAOConversion.RShouderRoll]
                                                                + NAOConversion.convertAngle(NAOConversion.RShouderRoll,shoulderRightRollAngle) / this.avg;

            //SHOULDERPITCH
            //Left
            SkeletonPoint hipLeftpos = skel.Joints[JointType.HipLeft].Position;
            float shoulderLeftPitchAngle = Kinematic.getAngle(hipLeftpos, shoulderLeftPos,
                                       elbowLeftPos, true);  
            float conv = NAOConversion.convertAngle(NAOConversion.LShoulderPitch,shoulderLeftPitchAngle);
            float avgd = conv / this.avg;
            this.jointAngles[NAOConversion.LShoulderPitch] = this.jointAngles[NAOConversion.LShoulderPitch]
                                                                +  avgd;
            //Right
            SkeletonPoint hipRightpos = skel.Joints[JointType.HipRight].Position;
            float shoulderRightPitchAngle = Kinematic.getAngle(hipRightpos, shoulderRightPos,
                                                            elbowRightPos, true);
            this.jointAngles[NAOConversion.RShoulderPitch] = this.jointAngles[NAOConversion.RShoulderPitch]
                                                               + NAOConversion.convertAngle(NAOConversion.RShoulderPitch,shoulderRightPitchAngle) / this.avg;

            //HIPROLL
            SkeletonPoint hipCenterPos = skel.Joints[JointType.HipCenter].Position;
            //Left
            
            SkeletonPoint kneeLeftPos = skel.Joints[JointType.KneeLeft].Position;
            float hipLeftRollAngle = Kinematic.getAngle(hipCenterPos, hipLeftpos, kneeLeftPos, false);
            //this.jointAngles[NAOConversion.LHipRoll] = this.jointAngles[NAOConversion.LHipRoll]
            //                                                   + NAOConversion.convertAngle(NAOConversion.LHipRoll,hipLeftRollAngle) / this.avg;
            //Right
            SkeletonPoint kneeRightPos = skel.Joints[JointType.KneeRight].Position;
            float hipRightRollAngle = Kinematic.getAngle(hipCenterPos, hipRightpos, kneeRightPos, false);
            //this.jointAngles[NAOConversion.RHipRoll] = this.jointAngles[NAOConversion.RHipRoll]
            //                                                  + NAOConversion.convertAngle(NAOConversion.RHipRoll,hipRightRollAngle) / this.avg;

            //HIPPITCH
            //Left
            float hipLeftPitchAngle = Kinematic.getAngle(this.calibrationValue[JointType.KneeLeft], hipLeftpos,
                                                      kneeLeftPos, true);
            //this.jointAngles[NAOConversion.LHipPitch] = this.jointAngles[NAOConversion.LHipPitch]
            //                                                   + NAOConversion.convertAngle(NAOConversion.LHipPitch,hipLeftPitchAngle) / this.avg;
            //Right
            float hipRightPitchAngle = Kinematic.getAngle(this.calibrationValue[JointType.KneeRight], hipRightpos,
                                                      kneeRightPos, true);
            //this.jointAngles[NAOConversion.RHipPitch] = this.jointAngles[NAOConversion.RHipPitch]
            //                                                  + NAOConversion.convertAngle(NAOConversion.RHipPitch,hipRightPitchAngle) / this.avg;

            //KNEEPITCH
            //Left
            SkeletonPoint ankleLeftPos = skel.Joints[JointType.AnkleLeft].Position;
            float kneeLeftPitchAngle = Kinematic.getAngle(hipLeftpos, kneeLeftPos, ankleLeftPos, false);
            this.jointAngles[NAOConversion.LKneePitch] = this.jointAngles[NAOConversion.LKneePitch]
                                                               + NAOConversion.convertAngle(NAOConversion.LKneePitch,kneeLeftPitchAngle) / this.avg;
            //Right
            SkeletonPoint ankleRightPos = skel.Joints[JointType.AnkleRight].Position;
            float kneeRightPitchAngle = Kinematic.getAngle(hipRightpos, kneeRightPos, ankleRightPos, false);
            this.jointAngles[NAOConversion.RKneePitch] = this.jointAngles[NAOConversion.RKneePitch]
                                                               + NAOConversion.convertAngle(NAOConversion.RKneePitch,kneeRightPitchAngle) / this.avg;


            //ANKLEPITCH
            //Left
            SkeletonPoint footLeftPos = skel.Joints[JointType.FootLeft].Position;
            float ankleLeftPitch = Kinematic.getAngle(kneeLeftPos, ankleLeftPos, footLeftPos, false);
            //this.jointAngles[NAOConversion.LAnklePitch] = this.jointAngles[NAOConversion.LAnklePitch]
                                                              //+ NAOConversion.convertAngle(NAOConversion.LAnklePitch,ankleLeftPitch) / this.avg;
            //Right
            SkeletonPoint footRightPos = skel.Joints[JointType.FootRight].Position;
            float ankleRightPitch = Kinematic.getAngle(kneeRightPos, ankleRightPos, footRightPos, false);
            //this.jointAngles[NAOConversion.RAnklePitch]  = this.jointAngles[NAOConversion.RAnklePitch]
                                                             //+ NAOConversion.convertAngle(NAOConversion.RAnklePitch,ankleRightPitch) / this.avg;



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

            motioProxy.wbEnable(true);
            motioProxy.wbFootState("Fixed", "Legs");
            motioProxy.wbEnableBalanceConstraint(true, "Legs");
            foreach (String x in NAOConversion.listOfTheJoint())
            {
                
                motioProxy.setAngles(x, this.jointAngles[x], fractionSpeed);
                this.jointAngles[x] = 0f;
            }
 
        }

        //----------------------------------Locomotion------------------------------------------

        private void movementFootStep(Skeleton skel)
        {
            this.footPosition[JointType.FootLeft] = JointUtilities.averagePoint(this.footPosition[JointType.FootLeft],
                                                                   skel.Joints[JointType.FootLeft].Position,
                                                                   this.avg);
            this.footPosition[JointType.FootRight] = JointUtilities.averagePoint(this.footPosition[JointType.FootRight],
                                                                   skel.Joints[JointType.FootRight].Position,
                                                                   this.avg);

           
        }

        private void footStepMovement(Skeleton skel)
        {
            //Get the right and left foot position then try to move the NAO
            //computing the distance along the Z and the X axis of the kinect which are
            //respectively the X and the Y axis on the NAO
            SkeletonPoint new_leftFootPos = this.footPosition[JointType.FootLeft];
            SkeletonPoint new_rightFootPos = this.footPosition[JointType.FootRight];

            SkeletonPoint old_leftFootPos = this.calibrationValue[JointType.FootLeft];
            SkeletonPoint old_rightFootPos = this.calibrationValue[JointType.FootRight];

            System.Collections.Generic.List<float> speed = new System.Collections.Generic.List<float>() { fractionSpeed };

            float stepX = NAOConversion.convertFootStepXAxis(new_leftFootPos.Z - new_rightFootPos.Z);
            float stepY = NAOConversion.converFootStepYAxis(new_leftFootPos.X - new_rightFootPos.X);

            System.Collections.ArrayList mov = new System.Collections.ArrayList() { stepX, stepY, 0f };


            if (JointUtilities.compareZXSkelPoint(new_leftFootPos, old_leftFootPos))
            {
                System.Collections.Generic.List<String> leg = new System.Collections.Generic.List<String>() { "LLeg" };
                System.Collections.ArrayList movl = new System.Collections.ArrayList() { mov };

                motioProxy.wbEnable(false);
                motioProxy.setFootStepsWithSpeed(leg, movl, speed, false);

                //Once the movement is done it is needed to recalibrate
                this.calibrationValue[JointType.FootLeft] = skel.Joints[JointType.FootLeft].Position;
            }
            else if (JointUtilities.compareZXSkelPoint(new_rightFootPos, old_rightFootPos))
            {

                System.Collections.Generic.List<String> leg = new System.Collections.Generic.List<String>() { "RLeg" };
                System.Collections.ArrayList movr = new System.Collections.ArrayList() { mov };

                motioProxy.wbEnable(false);
                motioProxy.setFootStepsWithSpeed(leg, movr, speed, false);

                //Once the movement is done it is needed to recalibrate
                this.calibrationValue[JointType.FootRight] = skel.Joints[JointType.FootRight].Position;
            }



        }

 


    }
}
