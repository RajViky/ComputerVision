#include <iostream>
#include <cstdlib>
#include <signal.h>

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>

using namespace cv;
using namespace std;

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


    std::string dir = "/tmp/images";
    boost::filesystem::path path = dir;
    if ( exists( path ) )
    {
        boost::filesystem::remove_all(path);
    }
    if (!boost::filesystem::create_directories(path / "rgb")
            || !boost::filesystem::create_directories(path / "depth")
            || !boost::filesystem::create_directories(path / "orig_rgb")
            || !boost::filesystem::create_directories(path / "depth_mat")
            || !boost::filesystem::create_directories(path / "pcd"))
        std::cout << "Cannot create tmp UNI DIR" << std::endl;

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


        boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
        boost::posix_time::ptime t1 = boost::posix_time::microsec_clock::local_time();
        double diff = (t1-epoch).total_milliseconds()/1000.0;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(4) << diff;
        std::string time = ss.str();


        cv::Mat mirror;
        cv::Mat converted;
        undistortedMat.convertTo(converted, CV_16UC1);
//        cv::flip(converted,mirror,1);
//        imwrite( dir +"/depth/"+time+".png", mirror );
//        cv::flip(registeredMat,mirror,1);
//        imwrite( dir +"/rgb/"+time+".png", mirror );
//        cv::flip(rgbMat,mirror,1);
//        imwrite( dir +"/orig_rgb/"+time+".jpg", mirror );

        cv::flip(converted,mirror,1);
        imwrite( dir +"/depth/tmp-depth.png", mirror );
        cv::flip(registeredMat,mirror,1);
        imwrite( dir +"/rgb/tmp-rgb.png", mirror );
        cv::flip(rgbMat,mirror,1);
        imwrite( dir +"/orig_rgb/tmp-orig_rgb.jpg", mirror );
        cv::flip(undistortedMat,mirror,1);
        mirror /= 1000;
        cv::FileStorage file( dir +"/depth_mat/tmp-depth_mat.xml", cv::FileStorage::WRITE);
        file << "depth_mat" << mirror;
        file.release();

        boost::filesystem::rename(dir +"/depth/tmp-depth.png",dir +"/depth/depth.png");
        boost::filesystem::rename(dir +"/rgb/tmp-rgb.png",dir +"/rgb/rgb.png");
        boost::filesystem::rename(dir +"/orig_rgb/tmp-orig_rgb.jpg",dir +"/orig_rgb/orig_rgb.jpg");
        boost::filesystem::rename(dir +"/depth_mat/tmp-depth_mat.xml",dir +"/depth_mat/depth_mat.xml");
        //usleep(500);


        listener.release(frames);
    }
    dev->stop();
    dev->close();

    delete registration;

    return 0;
}
