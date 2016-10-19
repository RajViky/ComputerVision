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


#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>


//planar segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>


//boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//  // --------------------------------------------
//  // -----Open 3D viewer and add point cloud-----
//  // --------------------------------------------
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();
//  return (viewer);
//}

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
    {
        viewer->setBackgroundColor (0, 0, 0);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
            viewer->removeAllPointClouds();
            viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

    }

    void run ()
    {
        pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

        //boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

        while (true)
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //pcl::visualization::CloudViewer viewer;
};

int main ()
{
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}
