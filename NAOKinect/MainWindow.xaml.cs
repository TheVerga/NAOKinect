using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace NAOKinect
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        Kinect kinect = new Kinect();
        ToNao nao;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            kinect.KinectConnected += disableButtonKinect;
            kinect.KinectDisconnected += enableButtonKinect;
            
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            kinect.Connect();
            kinect.StartSkeletonStream();
            SkeletonDrawer skeletonDrawer = new SkeletonDrawer(kinect.CoordinateMapper);
            kinect.SkeletonReady += skeletonDrawer.SensorSkeletonFrameReady;
            Image.Source = skeletonDrawer.getImageSource();
        }

        private void enableButtonKinect(string deviceConnectionId)
        {
            kinectButton.IsEnabled = true;
        }
        private void disableButtonKinect(string deviceConnectionId)
        {
            kinectButton.IsEnabled = false;
        }

        private void ConnectNAOClick(object sender, RoutedEventArgs e)
        {
            nao = new ToNao(IPBox.Text, int.Parse(PortBox.Text), kinect, 20);
            nao.setUpNao();
            nao.AnglesSent += startStreamAngles;
        }

        public void startStreamAngles(Dictionary<string, float> angles)
        {
            foreach (String x in NAOConversion.listOfTheJoint())
            {
                AngleBox.Text += x + " : " + angles[x].ToString();              
            }

            AngleBox.Text += Environment.NewLine;
        }
    }
}
