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
    /// faccio un commento per provare a committare (FV)
    /// </summary>
    public partial class MainWindow : Window
    {
        Kinect kinect = new Kinect();

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
    }
}
