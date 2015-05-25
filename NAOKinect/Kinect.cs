using Microsoft.Kinect;
using System;
using System.Collections;
using System.IO;

namespace NAOKinect
{
    public delegate void SkeletonReadyEventHandler(Skeleton[] skeletons);

    /// <summary>
    /// 
    /// </summary>
    class Kinect
    {
        private KinectSensor sensor = null;

        private ArrayList resources = new ArrayList(5);

        /// <summary>
        /// Event that fires when a new skeleton frame is available from this Kinect sensor.
        /// </summary>
        public event SkeletonReadyEventHandler SkeletonReady;

        /// <summary>
        /// Create a new <code>Kinect</code> class. To actually use the sensor, use <code>Connect()</code>
        /// </summary>
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

        /// <summary>
        /// Gets a value indicating whether the Kinect is currently connected.
        /// </summary>
        public bool IsConnected
        {
            get
            {
                return null != sensor && KinectStatus.Connected == sensor.Status;
            }
        }

        /// <summary>
        /// Gets a value indicating whether the Kinect is currently streaming data.
        /// </summary>
        public bool IsActive
        {
            get
            {
                return null != sensor && sensor.IsRunning;
            }
        }

        /// <summary>
        /// Try to connect to an attached Kinect sensor. It connects to the first available sensor, thus
        /// it does not support multiple devices. This API throws a System.IO.IOException if no sensor
        /// is connected.
        /// </summary>
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

        /// <summary>
        /// Start streaming skeleton data from this Kinect. If the sensor is off, it is activated.
        /// To receive notifications about the skeleton frames, add an handler to the SkeletonReady event.
        /// This API may throw a System.IO.IOException if the current sensor is already in use 
        /// by another process.
        /// </summary>
        public void StartSkeletonStream()
        {
            if (null != sensor)
            {
                Start();
                sensor.SkeletonStream.Enable();
                resources.Add(sensor.SkeletonStream);
                sensor.SkeletonFrameReady += OnSkeletonFrameReady;

                //Due to a bug, enabling the SkeletonStream turns off the AudioSource.
                //If the AudioSource was active, it must be restarted.
                if (resources.Contains(sensor.AudioSource))
                {
                    sensor.AudioSource.Start();
                }
            }
        }

        /// <summary>
        /// Stop streaming skeleton data from this Kinect. If nothing else is using this sensor, 
        /// the Kinect is switched off.
        /// </summary>
        public void StopSkeletonStream()
        {
            if (null != sensor)
            {
                sensor.SkeletonStream.Disable();
                resources.Remove(sensor.SkeletonStream);
                sensor.SkeletonFrameReady -= OnSkeletonFrameReady;
                Stop();
            }
        }

        private void Start()
        {
            if (!IsActive)
            {
                sensor.Start();
            }
        }

        private void Stop()
        {
            if (IsActive && resources.Count == 0)
            {
                sensor.Stop();
            }
        }

        private void OnSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
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
