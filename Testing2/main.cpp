/* \author Geoffrey Biggs */

#include <iostream>
#include <cstdlib>
#include <signal.h>

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <fstream>
#include <cstdlib>

#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include <Eigen/StdVector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni2_grabber.h>


#include <pcl/io/lzf_image_io.h>



#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace cv;
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr Mat32FToPoinXYZ(cv::Mat depthMat)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZ>);

    // calibration parameters
    float const fx_d = 5.9421434211923247e+02;
    float const fy_d = 5.9104053696870778e+02;
    float const cx_d = 3.3930780975300314e+02;
    float const cy_d = 2.4273913761751615e+02;

    unsigned char* p = depthMat.data;
    for (int i = 0; i<depthMat.rows; i++)
    {
        for (int j = 0; j < depthMat.cols; j++)
        {
            float val=0;
            unsigned n = (unsigned)((*p) | (*(++p) << 8) | (*(++p) << 16) | *(++p) << 24);
            memcpy(&val,&n,4);

            pcl::PointXYZ point;
            point.z = 0.001 * val;
            point.x = point.z*(j - cx_d)  / fx_d;
            point.y = point.z *(cy_d - i) / fy_d;
            ptCloud->points.push_back(point);
            ++p;
        }
    }
    ptCloud->width = (int)depthMat.cols;
    ptCloud->height = (int)depthMat.rows;

    return ptCloud;

}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Mat32FToPoinXYZRGBA(cv::Mat depthMat,cv::Mat rgbMat)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    // calibration parameters
    float const fx_d = 5.9421434211923247e+02;
    float const fy_d = 5.9104053696870778e+02;
    float const cx_d = 3.3930780975300314e+02;
    float const cy_d = 2.4273913761751615e+02;

    unsigned char* p = depthMat.data;
    unsigned char* q = rgbMat.data;
    for (int i = 0; i<depthMat.rows; i++)
    {
        for (int j = 0; j < depthMat.cols; j++)
        {
            float val=0;
            unsigned n = (unsigned)((*p) | (*(++p) << 8) | (*(++p) << 16) | *(++p) << 24);
            memcpy(&val,&n,4);

            pcl::PointXYZRGBA point;
            point.z = 0.001 * val;
            point.x = point.z*(j - cx_d)  / fx_d;
            point.y = point.z *(cy_d - i) / fy_d;
            point.b = *q++;
            point.g = *q++;
            point.r = *q++;
            point.a = *q++;
            ptCloud->points.push_back(point);
            ++p;
        }
    }
    ptCloud->width = (int)depthMat.cols;
    ptCloud->height = (int)depthMat.rows;

    return ptCloud;

}
bool protonect_shutdown = false;
void sigint_handler(int)
{
    protonect_shutdown = true;
}


int main()
{
    signal(SIGINT,sigint_handler);

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    std::string serial = "";

    size_t framemax = -1;
    //framemax = 1;


    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    if(pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    int types = 0;
    types |= libfreenect2::Frame::Color;
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);


    if (!dev->start())
        return -1;


    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    size_t framecount = 0;

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr data;
    pcl::PointCloud<pcl::PointXYZ>::Ptr data;

    /// [loop start]

    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
        std::cout << "timeout!" << std::endl;
        return -1;
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    registration->apply(rgb, depth, &undistorted, &registered);

    framecount++;

    cv::Mat rgbMat;
    cv::Mat depthMat;
    cv::Mat undistortedMat;
    cv::Mat registeredMat;

    cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbMat);
    cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthMat);


    cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(undistortedMat);
    cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(registeredMat);


    data = Mat32FToPoinXYZ(undistortedMat);
    //data = Mat32FToPoinXYZRGBA(undistortedMat,registeredMat);

    dev->stop();
    dev->close();
    boost::posix_time::ptime t1 = boost::posix_time::second_clock::local_time();
    boost::posix_time::ptime t2 = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = t2 - t1;

    // All the objects needed
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    // Datasets
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Read in the cloud data
//    reader.read ("../PCL-Cylinder-segmentation/table_scene_mug_stereo_textured.pcd", *cloud);
//    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;


    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (data);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
    t2 = boost::posix_time::second_clock::local_time();
    diff = t2 - t1;
    std::cout << diff.minutes() << ":" << std::setw(2) << std::setfill('0') << diff.seconds() << "." << diff.total_milliseconds()-1000*(diff.seconds() + 60*diff.minutes()) << std::endl;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
    t2 = boost::posix_time::second_clock::local_time();
    diff = t2 - t1;
    std::cout << diff.minutes() << ":" << std::setw(2) << std::setfill('0') << diff.seconds() << "." << diff.total_milliseconds()-1000*(diff.seconds() + 60*diff.minutes()) << std::endl;


    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    // Write the planar inliers to disk
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    t2 = boost::posix_time::second_clock::local_time();
    diff = t2 - t1;
    std::cout << diff.minutes() << ":" << std::setw(2) << std::setfill('0') << diff.seconds() << "." << diff.total_milliseconds()-1000*(diff.seconds() + 60*diff.minutes()) << std::endl;


    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    t2 = boost::posix_time::second_clock::local_time();
    diff = t2 - t1;
    std::cout << diff.minutes() << ":" << std::setw(2) << std::setfill('0') << diff.seconds() << "." << diff.total_milliseconds()-1000*(diff.seconds() + 60*diff.minutes()) << std::endl;


    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
        writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }
    t2 = boost::posix_time::second_clock::local_time();
    diff = t2 - t1;
    std::cout << diff.minutes() << ":" << std::setw(2) << std::setfill('0') << diff.seconds() << "." << diff.total_milliseconds()-1000*(diff.seconds() + 60*diff.minutes()) << std::endl;




    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (1.0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud_filtered2, "sample cloud");
    //viewer.addPointCloud<pcl::PointXYZ> (cloud_f, single_colorR,"f");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorG(cloud_plane, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud_plane, single_colorG,"planes");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorR(cloud_cylinder, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud_cylinder, single_colorR,"cylinder");

    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    viewer.spin();
    while(true)
    {
        usleep(100);
    }
    return (0);
}
