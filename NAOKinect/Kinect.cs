using System;
using System.IO;
using Microsoft.Kinect;

namespace NAOKinect
{
    public delegate void SkeletonReadyEventHandler(Skeleton[] skeletons);

    class Kinect
    {
        private KinectSensor sensor = null;

        private bool isSkeletonActive = false;
        private bool isAudioActive = false;

        public event SkeletonReadyEventHandler SkeletonReady;

        public Kinect()
        {
            KinectSensor.KinectSensors.StatusChanged += KinectSensors_StatusChanged;
        }

        public ~Kinect()
        {
            if (sensor != null)
            {
                sensor.Stop();
            }
        }

        public bool IsConnected
        {
            get
            {
                return null != sensor && KinectStatus.Connected == sensor.Status;
            }
        }

        public bool IsActive
        {
            get
            {
                return null != sensor && sensor.IsRunning;
            }
        }

        public void Connect()
        {
            foreach (KinectSensor kinectSensor in KinectSensor.KinectSensors)
            {
                if (KinectStatus.Connected == kinectSensor.Status)
                {
                    sensor = kinectSensor;
                    break;
                }
            }
            throw new IOException("No kinect sensor found.");
        }

        public void StartSkeletonStream()
        {
            if (null != sensor)
            {
                sensor.SkeletonStream.Enable();
                isSkeletonActive = true;
                sensor.SkeletonFrameReady += sensor_SkeletonFrameReady;
                Start();

                //Due to a bug, enabling the SkeletonStream turns off the AudioSource.
                //If the AudioSource was active, it must be restarted.
                if (isAudioActive)
                {
                    sensor.AudioSource.Start();
                }
            }
        }

        public void StopSkeletonStream()
        {
            if (null != sensor)
            {
                sensor.SkeletonStream.Disable();
                isSkeletonActive = false;
                sensor.SkeletonFrameReady -= sensor_SkeletonFrameReady;
            }
        }

        private void Start()
        {
            if (!IsActive)
            {
                sensor.Start();
            }
        }

        public void Stop()
        {
            if (IsActive)
            {
                sensor.Stop();
            }
        }

        private void sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            if (SkeletonReady != null)
            {
                SkeletonReady(skeletons);
            }
        }

        private void KinectSensors_StatusChanged(object sender, StatusChangedEventArgs e)
        {
            if (sensor == e.Sensor)
            {

            }
        }
    }
}
