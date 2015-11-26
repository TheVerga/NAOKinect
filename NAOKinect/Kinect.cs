using Microsoft.Kinect;
using Microsoft.Speech.Recognition;
using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Linq;

namespace NAOKinect
{
    public delegate void SkeletonReadyEventHandler(Skeleton[] skeletons);

    public delegate void SpeechRecognizedEventHandler(SpeechRecognizedEventArgs e);

    public delegate void KinectDisconnectedEventHandler(string deviceConnectionId);

    public delegate void KinectConnectedEventHandler(string deviceConnectionId);

    /// <summary>
    /// A class representing a Microsoft Kinect sensor. It allows using the sensor without having to
    /// handle low level features.
    /// </summary>
    public class Kinect
    {
        private KinectSensor sensor = null;

        private SpeechRecognitionEngine speechRecognizer = null;

        private float speechRecognitionConfidence = 0.0f;

        //Used to keep track of the streaming resources the user is using (such as Skeleton, Audio, etc.)
        private ArrayList resources = new ArrayList(5);

        /// <summary>
        /// Event that fires when a new skeleton frame is available from this Kinect sensor.
        /// </summary>
        public event SkeletonReadyEventHandler SkeletonReady;
        
        /// <summary>
        /// Event that fires when the sensor matches some speech with the provided grammar.
        /// </summary>
        public event SpeechRecognizedEventHandler SpeechRecognized;

        /// <summary>
        /// Event that fires when the current kinect sensor is no longer connected.
        /// </summary>
        public event KinectDisconnectedEventHandler KinectDisconnected;

        /// <summary>
        /// Event that fires when a new kinect sensor is connected.
        /// </summary>
        public event KinectConnectedEventHandler KinectConnected;

        /// <summary>
        /// Create a new Kinect class. To actually use the sensor, use <c>Connect()</c>.
        /// </summary>
        public Kinect()
        {
            KinectSensor.KinectSensors.StatusChanged += OnStatusChanged;
        }

        ~Kinect()
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
        /// Gets the coordinate mapper associated to the current sensor. If no sensor is connected, 
        /// it returns <c>null</c>
        /// </summary>
        public CoordinateMapper CoordinateMapper
        {
            get
            {
                return (null == sensor)? null : sensor.CoordinateMapper;
            }
        }

        /// <summary>
        /// Try to connect to an attached Kinect sensor. It connects to the first available sensor and
        /// it does not support multiple devices. This API throws a System.IO.IOException if no sensor
        /// is connected.
        /// </summary>
        /// <exception cref="System.IO.IOException">If no sensor is found.</exception>
        public void Connect()
        {
            //Search for a kinect and connect to the first available.
            sensor = (

                from kinectSensor in KinectSensor.KinectSensors
                where KinectStatus.Connected == kinectSensor.Status
                select kinectSensor

                ).FirstOrDefault();

            if (sensor == null)
            {
                throw new IOException("No kinect sensor found.");
            }

            //Try to get the speech recognizer with the same culture as the user interface.
            RecognizerInfo recognizerInfo = GetKinectRecognizer(CultureInfo.CurrentUICulture.Name);

            //If the recognizer is found, create a speech recognition engine that uses that recognizer.
            if (null != recognizerInfo)
            {
                speechRecognizer = new SpeechRecognitionEngine(recognizerInfo.Id);
            }
        }

        /// <summary>
        /// Try to connect to the Kinect sensor specified by the <paramref name="deviceConnectionId"/>.
        /// It does not support multiple devices. This API throws a System.IO.IOException if no sensor
        /// with the specified <paramref name="deviceConnectionId"/> is connected.
        /// </summary>
        /// <param name="deviceConnectionId">The DeviceConnectionId of the Kinect to connect.</param>
        /// <exception cref="System.IO.IOException">
        /// If no sensor with the specified <paramref name="deviceConnectionId"/> is found.
        /// </exception>
        public void Connect(string deviceConnectionId)
        {
            //Select the kinect with the specified DeviceConnectionId. If not found return null.
            sensor = (

                from kinectSensor in KinectSensor.KinectSensors
                where KinectStatus.Connected == kinectSensor.Status
                    && deviceConnectionId == kinectSensor.DeviceConnectionId
                select kinectSensor

                ).FirstOrDefault();

            if (sensor == null)
            {
                throw new IOException("No kinect sensor found with ConnectionId = "
                    + deviceConnectionId + ".");
            }

            //Try to get the speech recognizer with the same culture as the user interface.
            RecognizerInfo recognizerInfo = GetKinectRecognizer(CultureInfo.CurrentUICulture.Name);

            //If the recognizer is found, create a speech recognition engine that uses that recognizer.
            if (null != recognizerInfo)
            {
                speechRecognizer = new SpeechRecognitionEngine(recognizerInfo.Id);
            }
        }

        /// <summary>
        /// Start streaming skeleton data from this Kinect. If the sensor is off, it is activated.
        /// To receive notifications about the skeleton frames, add an handler to the SkeletonReady event.
        /// This API may throw a System.IO.IOException if no kinect sensor is connected or if the current
        /// sensor is already in use by another process.
        /// </summary>
        /// <exception cref="System.IO.IOException">
        /// If no kinect is connected or if the current sensor is in use by another process.
        /// </exception>
        public void StartSkeletonStream()
        {
            if (null == sensor)
            {
                throw new IOException("No kinect is connected. Try using Kinect.Connect().");
            }
            if (!resources.Contains(sensor.SkeletonStream))
            {
                TryStart();
                sensor.SkeletonStream.Enable(new TransformSmoothParameters()
                {
                    Smoothing = 0.75f,
                    Correction = 0.0f,
                    Prediction = 0.0f,
                    JitterRadius = 0.05f,
                    MaxDeviationRadius = 0.04f
                });

                //Add the skeleton stream to the currently active streaming resources.
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
        /// Stop streaming skeleton data from this Kinect. If no other streaming is active on this sensor, 
        /// the Kinect is switched off.
        /// </summary>
        public void StopSkeletonStream()
        {
            if (null != sensor)
            {
                sensor.SkeletonStream.Disable();

                //Remove the skeleton stream from the currently active streaming resources.
                resources.Remove(sensor.SkeletonStream);

                sensor.SkeletonFrameReady -= OnSkeletonFrameReady;
                TryStop();
            }
        }

        /// <summary>
        /// Start recognizing speech. It matches the input audio stream against the specified
        /// <paramref name="grammar"/>, when it recognizes some speech the SpeechRecognized event is
        /// fired. The <paramref name="confidence"/> parameter specifies the minimum certainty threshold
        /// above which the event must be thrown. Generally speaking, confidence expresses how much 'sure'
        /// the recognition engine is that the recognized phrase is actually what it was pronounced.
        /// It ranges from 0 (meaning 0%) to 1 (meaning 100%).
        /// </summary>
        /// <param name="grammar">The grammar to match</param>
        /// <param name="confidence">The level of confidence above which the SpeechRecognized
        /// event is fired. It accepts values from 0 (inclusive) to 1 (inclusive).
        /// </param>
        /// <remarks>
        /// NOTE: A low confidence is likely to fire lot of events even for wrong recognitions. Conversely,
        /// a high confidence will fire few events and discard correct recognitions.
        /// </remarks>
        public void StartAudioRecognition(Grammar grammar, float confidence)
        {
            if (null == sensor)
            {
                throw new IOException("No kinect is connected. Try using Kinect.Connect().");
            }

            if (null == speechRecognizer)
            {
                throw new InvalidOperationException(@"Impossible to start recognizing. No kinect 
                speech recognizer found.\nTry to Connect() again.");
            }

            if (!resources.Contains(sensor.AudioSource))
            {
                TryStart();

                speechRecognizer.SpeechRecognized += OnSpeechRecognized;

                //Pass the input grammar to the recognizer.
                speechRecognizer.LoadGrammar(grammar);

                //Set the minimum confidence that fires the SpeechRecognized event.
                setRecognitionConfidence(confidence);

                sensor.AudioSource.Start();

                //Add the audio stream to the currently active streaming resources.
                resources.Add(sensor.AudioSource);

                speechRecognizer.SetInputToDefaultAudioDevice();

                //Start recognizing.
                speechRecognizer.RecognizeAsync(RecognizeMode.Multiple);
            }
        }

        /// <summary>
        /// Stop recognizing speech with the current Kinect. If no other streaming is active, it
        /// stops the sensor.
        /// </summary>
        public void StopAudioRecognition()
        {
            if (null != sensor && null != speechRecognizer)
            {
                speechRecognizer.RecognizeAsyncStop();

                sensor.AudioSource.Stop();

                //Remove the audio stream from the currently active streaming resources.
                resources.Remove(sensor.AudioSource);

                TryStop();
            }
        }

        /// <summary>
        /// Force the kinect to use the specified speech recognizer instead of the default one. The
        /// default recognizer is the one of the same culture as the user interface of the program.
        /// This API may throw a Microsoft.Speech.Recognition.InvalidCultureException if no speech
        /// recognizer with the specified culture is found.
        /// </summary>
        /// <param name="cultureName">
        /// The culture name of the speech recognizer to obtain. The string must be in the form
        /// languagecode2-country/regioncode2. E.g. "en-US" or "it-IT".
        /// </param>
        /// <exception cref="Microsoft.Speech.Recognition.InvalidCultureException">
        /// If no speech recognizer with the specified culture is found.
        /// </exception>
        public void UseRecognizer(string cultureName)
        {
            RecognizerInfo tempRecognizer = GetKinectRecognizer(cultureName);
            if (tempRecognizer == null)
            {
                throw new InvalidCultureException(@"No recognizer with the specified culture was found.\nSpecified culture: " + cultureName);
            }
            else
            {
                speechRecognizer = new SpeechRecognitionEngine(tempRecognizer.Id);
            }
        }

        private void TryStart()
        {
            if (!IsActive)
            {
                sensor.Start();
            }
        }

        private void TryStop()
        {
            if (IsActive && resources.Count == 0)
            {
                sensor.Stop();
            }
        }

        private void setRecognitionConfidence(float confidence)
        {
            //Check if confidence is between 0 and 1. If not, the default value 0 is set.
            if (confidence >= 0.0f && confidence <= 1.0f)
            {
                speechRecognitionConfidence = confidence;
            }
            else
            {
                confidence = 0.0f;
            }
        }

        private RecognizerInfo GetKinectRecognizer(string cultureName)
        {
            //Search for a kinect speech recognizer and use the first available.
            foreach (RecognizerInfo recognizer in SpeechRecognitionEngine.InstalledRecognizers())
            {
                string value = null;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                if ("true".Equals(value, StringComparison.OrdinalIgnoreCase) &&
                    cultureName == recognizer.Culture.Name)
                {
                    return recognizer;
                }
            }
            return null;
        }

        private void OnSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            //Extract the skeletons from the param e, to the skeletons array
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (null != skeletonFrame)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            //If some function is attached to the event
            if (null != SkeletonReady)
            {
                //Notify passing the tracked skeletons
                SkeletonReady(skeletons);
            }
        }

        private void OnSpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            if (e.Result.Confidence >= speechRecognitionConfidence)
            {
                //If some function is attached to the event
                if (null != SpeechRecognized)
                {
                    //Notify passing the SpeechRecognizedEventArgs param e.
                    SpeechRecognized(e);
                }
            }
        }

        private void OnStatusChanged(object sender, StatusChangedEventArgs e)
        {
            if (sensor == e.Sensor)
            {
                if (KinectStatus.Disconnected == e.Status)
                {
                    sensor = null;
                    resources.Clear();
                    KinectDisconnected(e.Sensor.DeviceConnectionId);
                }
            }
            else
            {
                if (KinectStatus.Connected == e.Status)
                {
                    KinectConnected(e.Sensor.DeviceConnectionId);
                }
            }
        }
    }
}
