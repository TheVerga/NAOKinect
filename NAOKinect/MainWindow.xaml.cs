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
        public MainWindow()
        {
            InitializeComponent();
        }

        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            Kinect kinect = new Kinect();
            kinect.Connect();
            SkeletonDrawer skeletonDrawer = new SkeletonDrawer(kinect.CoordinateMapper);
           
            Image.Source = skeletonDrawer.getImageSource();

            kinect.SkeletonReady += skeletonDrawer.SensorSkeletonFrameReady;

            kinect.StartSkeletonStream();

        }
    }
}
