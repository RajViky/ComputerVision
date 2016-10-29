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
#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr Mat16UToPoinXYZ(cv::Mat depthMat)
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
            unsigned short n = (unsigned short)((*p) | (*(++p) << 8));

            pcl::PointXYZ point;
            point.z = 0.001 * n;
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
pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ_TEST(cv::Mat depthMat)
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
            float z = static_cast<float>(*p);
            pcl::PointXYZ point;
            point.z = 0.1 * z;
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
pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat depthMat)
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
            float z = static_cast<float>(*p);
            pcl::PointXYZ point;
            point.z = 0.001 * z;
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
bool protonect_shutdown = false;
void sigint_handler(int s)
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


    /// [loop start]
    pcl::visualization::CloudViewer viewer("Viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr data;

    while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
    {
        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
        {
            std::cout << "timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered);

        framecount++;

        cv::Mat rgbMat;
        cv::Mat irMat;
        cv::Mat depthMat;
        cv::Mat undistortedMat;
        cv::Mat registeredMat;

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbMat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irMat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthMat);


        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(undistortedMat);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(registeredMat);


        imwrite( "/tmp/rgbMat.png", rgbMat );
        imwrite( "/tmp/depthMat.png", depthMat );
        cv::Mat aaa;
        undistortedMat.convertTo(aaa, CV_16UC1);
        //undistortedMat.convertTo(aaa, CV_8UC1);
        imwrite( "/tmp/undistortedMat.png", undistortedMat );
        imwrite( "/tmp/test.png", aaa );
        imwrite( "/tmp/registeredMat.png", registeredMat );


        //cv::Mat newMat = imread( "/tmp/test.png",0);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr test = MatToPoinXYZ_TEST(newMat);

        //POINT CLOUD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        //pcl::PointCloud<pcl::PointXYZ>::Ptr test = MatToPoinXYZ(newMat);

        cv::Mat newMat = imread( "/tmp/test.png",CV_LOAD_IMAGE_ANYDEPTH);
        data = Mat16UToPoinXYZ(newMat);

        //pcl::PointCloud<pcl::PointXYZ>::Ptr test = Mat32FToPoinXYZ(undistortedMat);
//        pcl::PointCloud<pcl::PointXYZ> test1;
//        const string f = "/tmp/test.png";
//        pcl::io::LZFDepth16ImageReader reader;
//        reader.read(f,test1);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr test(&test1);
        viewer.showCloud(data);

        listener.release(frames);
    }
    dev->stop();
    dev->close();

    pcl::PCDWriter writer;
    writer.write ("/tmp/test.pcd", *data, false);

    delete registration;

    return 0;
}
