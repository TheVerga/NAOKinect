using Microsoft.Kinect;
using System;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;

namespace NAOKinect
{
    class SkeletonRecorder
    {
        private const string defaultFileName = "skeletonRecording";
        private const string defaultFolderPath = "";
        private const string fileExtension = ".skrec";

        private string baseFileName;
        private string folderPath;

        private BinaryFormatter formatter = new BinaryFormatter();
        private Stream stream;

        private bool isRecording = false;

        public string BaseFileName
        {
            get
            {
                return baseFileName;
            }
            set
            {
                baseFileName = value;
            }
        }

        public string FolderPath
        {
            get
            {
                return folderPath;
            }
            set
            {
                folderPath = value;
            }
        }

        public bool IsRecording
        {
            get
            {
                return isRecording;
            }
        }

        public SkeletonRecorder()
        {
            baseFileName = defaultFileName;
            folderPath = defaultFolderPath;
        }

        ~SkeletonRecorder()
        {
            if (null != stream)
            {
                stream.Close();
            }
        }

        public void Start()
        {
            string dateSnapshot = DateTime.Now.ToString("yyyyMMddHHmmss");
            string fileName = baseFileName + dateSnapshot + fileExtension;
            string fullPath = Path.Combine(folderPath, fileName);
            fullPath = Path.GetFullPath(fullPath);
            
            try
            {
                stream = new FileStream(fullPath, FileMode.CreateNew);
            }
            catch (Exception ex)
            {
                throw new IOException(ex.Message, ex);
            }

            isRecording = true;
        }

        public void Stop()
        {
            isRecording = false;

            if (null != stream)
            {
                stream.Close();
            }

            stream = null;
        }

        public void OnSkeletonFrame(Skeleton[] skeletons)
        {
            if (isRecording)
            {
                foreach (Skeleton skeleton in skeletons)
                {
                    if (SkeletonTrackingState.Tracked == skeleton.TrackingState)
                    {
                        formatter.Serialize(stream, skeleton);
                    }
                }
            }
        }
    }
}
