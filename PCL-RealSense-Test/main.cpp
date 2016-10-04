#include <pcl/io/real_sense_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

class SimpleRealSenseViewer
{
  public:
    SimpleRealSenseViewer () : viewer ("PCL OpenNI Viewer") {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::RealSenseGrabber();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&SimpleRealSenseViewer::cloud_cb_, this, _1);

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
  SimpleRealSenseViewer v;
  v.run ();
  return 0;
}
