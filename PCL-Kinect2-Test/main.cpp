//#include <pcl/io/openni2_grabber.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/console/parse.h>
//#include <iostream>

//using namespace std;
//using namespace pcl;

//int main(int argc, char** argv)
//{
//    PointCloud<PointXYZRGBA>::Ptr sourceCloud(new PointCloud<PointXYZRGBA>);

//    boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr&)> function = [&sourceCloud](const PointCloud<PointXYZRGBA>::ConstPtr &cloud)
//    {
//        copyPointCloud(*cloud, *sourceCloud);
//    };

//    // Create Kinect2Grabber
//    Grabber* grabber = new io::OpenNI2Grabber();
//    // Regist Callback Function
//    grabber->registerCallback(function);
//    // Start Retrieve Data
//    grabber->start();
//    boost::this_thread::sleep(boost::posix_time::seconds(1));
//    // Stop Retrieve Data
//    grabber->stop();
//    cout << "The Cloud size: " << sourceCloud->size() << " points ..." << endl;
//}


//#define _GLIBCXX_USE_CXX11_ABI 0
#include <Eigen/StdVector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni2_grabber.h>

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

      interface->registerCallback (f);

      interface->start ();

      while (!viewer.wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }

      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}
