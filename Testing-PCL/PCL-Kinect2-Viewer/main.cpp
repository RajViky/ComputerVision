#include <Eigen/StdVector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni2_grabber.h>

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    //void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
        {
            viewer.showCloud(cloud);
        }
    }

    void run ()
    {
        pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
        //boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

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
