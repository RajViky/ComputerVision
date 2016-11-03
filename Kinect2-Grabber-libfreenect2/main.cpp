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

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/date_time.hpp>

using namespace cv;
using namespace std;

bool protonect_shutdown = false;
void sigint_handler(int)
{
    protonect_shutdown = true;
}


int main(int argc, char **argv)
{
    //Parameters parsing
    std::vector<std::string> args(argv, argv + argc);
    args.erase(args.begin());
    bool help = false;
    std::string dir = "";
    std::string store_all = "";
    bool orig = false;
    bool depth_mat = false;
    float freq = 0;
    size_t framemax = -1;

    for(std::string arg : args)
    {
        if(arg.at(0) != '-' || arg == "help" || arg == "-help" || arg == "--help")
        {
            help = true;
            break;
        }
        arg.erase(arg.begin(),arg.begin()+1);
        if(arg.substr(0,5) == "clean")
        {
            std::cout << "Cleaning after unpropper exit." << std::endl;
            sigint_handler(0);
            break;
        }
        else if(arg.substr(0,3) == "dir")
        {
            dir = arg.substr(4,arg.length()-4);
            std::cout << "Main path:   " << dir << ";" << std::endl;
        }
        else if(arg.substr(0,1) == "f")
        {
            freq = std::stof(arg.substr(2,arg.length()-2));
            std::cout << "Frequency:   " << freq << "Hz;" << std::endl;
        }
        else if(arg.substr(0,4) == "orig")
        {
            orig = true;
            std::cout << "Original images will be stored." << std::endl;
        }
        else if(arg.substr(0,9) == "depth_mat")
        {
            depth_mat = true;
            std::cout << "Depth matrixes will be stored." << std::endl;
        }
        else if(arg.substr(0,9) == "store_all")
        {
            store_all = arg.substr(10,arg.length()-10);
            std::cout << "All data path:   " << store_all << ";" << std::endl;
        }
    }
    if(store_all == "")
        std::cout << "Only current images will be stored." << std::endl;

    if(help)
    {
        std::cout << "Kinect 2 image grabber" << std::endl;
        std::cout << "======================" << std::endl;
        std::cout << "Program should be closed with Ctrl+C" << std::endl;
        std::cout << "Available Parameters:" << std::endl;
        std::cout << "-dir          -dir=/tmp/grabber" << std::endl;
        std::cout << "     Base tmp directory where data is stored." << std::endl;
        std::cout << "     Path WILL BE CLEARED if it does exists." << std::endl;
        std::cout << "     Path WILL BE CREATED if it does not exists." << std::endl;
        std::cout << "     If not set /tmp/kinect2 is used." << std::endl;
        std::cout << "-f            -f=60" << std::endl;
        std::cout << "     Capture frequency in Hz." << std::endl;
        std::cout << "     If not set or set to 0, data is stored as fast as possible." << std::endl;
        std::cout << "-store_all    -store_all=/path/data" << std::endl;
        std::cout << "     If set: all images will be stored with timestamps in filenames to pat" << std::endl;
        std::cout << "     Path WILL NOT BE CLEARED if it does exists." << std::endl;
        std::cout << "     Path WILL BE CREATED if it does not exists." << std::endl;
        std::cout << "     If not set: only current images will be stored." << std::endl;
        std::cout << "-max_frames   -max_frames=1000" << std::endl;
        std::cout << "     Limits maximum number of recorded frames." << std::endl;
        std::cout << "     Applicatin will exit when limit is reached." << std::endl;
        std::cout << "     Unlimited if not set." << std::endl;
        std::cout << " --------------------------------------" << std::endl;
        std::cout << "Depth images and rgb images mapped to dept are always stored." << std::endl;
        std::cout << " " << std::endl;
        std::cout << "-depth_mat" << std::endl;
        std::cout << "     Dept data in cv::Mat CV_32FC1 is also stored." << std::endl;
        std::cout << "-orig" << std::endl;
        std::cout << "     RGB images in original resolution are also stored." << std::endl;
        std::cout << " --------------------------------------" << std::endl;
        std::cout << " -clean" << std::endl;
        std::cout << "     Run clenup after unpropper exit (Kinect is still recording - LED on)" << std::endl;
        std::cout << " -help" << std::endl;
        std::cout << "     Print this help." << std::endl;
        return 0;
    }

    if(dir == "")
        dir = "/tmp/kinect2";

    try
    {
        boost::filesystem::path path = dir;
        if ( exists( path ) )
        {
            boost::filesystem::remove_all(path);
        }
        if (!boost::filesystem::create_directories(path / "rgb")
                || !boost::filesystem::create_directories(path / "depth")
                || !boost::filesystem::create_directories(path / "orig_rgb")
                || !boost::filesystem::create_directories(path / "depth_mat"))
        {
            std::cout << "Cannot create tmp directories" << std::endl;
            return -1;
        }

        if(store_all != "")
        {
            path = store_all;
            if ((!boost::filesystem::is_directory(path / "rgb") && !boost::filesystem::create_directories(path / "rgb"))
                    || (!boost::filesystem::is_directory(path / "depth") && !boost::filesystem::create_directories(path / "depth"))
                    || (!boost::filesystem::is_directory(path / "orig_rgb") && !boost::filesystem::create_directories(path / "orig_rgb"))
                    || (!boost::filesystem::is_directory(path / "depth_mat") && !boost::filesystem::create_directories(path / "depth_mat")))
            {
                std::cout << "Cannot create store_all directories" << std::endl;
                return -1;
            }
        }
    }
    catch(std::exception e)
    {
        std::cout << "Cannot create required directories, exit." << std::endl;
        std::cout << e.what() << std::endl;
        return -1;
    }

    //^C
    signal(SIGINT,sigint_handler);
    //abort()
    signal(SIGABRT, sigint_handler);
    //sent by "kill" command
    signal(SIGTERM, sigint_handler);
    //^Z
    signal(SIGTSTP, sigint_handler);

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;

    std::string serial = "";

    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    serial = freenect2.getDefaultDeviceSerialNumber();
    dev = freenect2.openDevice(serial);

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

    boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    boost::posix_time::ptime last_image = boost::posix_time::microsec_clock::local_time();


    while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
    {
        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 seconds
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


        boost::posix_time::ptime t1 = boost::posix_time::microsec_clock::local_time();
        if(freq > 0.0)
        {
            int wanted_diff = 1000.0/freq;
            int last_diff = (t1-last_image).total_milliseconds();
            if(last_diff<wanted_diff)
                usleep((wanted_diff-last_diff)*1000);
            //std::cout << "Sleep: " <<  wanted_diff << " - " << last_diff << " = " << (wanted_diff - last_diff) << std::endl;
            t1 = boost::posix_time::microsec_clock::local_time();
        }
        double diff = (t1-epoch).total_milliseconds()/1000.0;
        int last_diff = (t1-last_image).total_milliseconds();
        std::cout << last_diff << " ms from last image = " << 1000.0/(float)last_diff << "Hz" << std::endl;
        last_image = t1;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(4) << diff;
        std::string time = ss.str();


        cv::Mat mirror;
        cv::Mat converted;
        undistortedMat.convertTo(converted, CV_16UC1);

        cv::flip(converted,mirror,1);
        imwrite( dir +"/depth/tmp-depth.png", mirror );
        cv::flip(registeredMat,mirror,1);
        imwrite( dir +"/rgb/tmp-rgb.png", mirror );
        if(orig)
        {
            cv::flip(rgbMat,mirror,1);
            imwrite( dir +"/orig_rgb/tmp-orig_rgb.jpg", mirror );
        }
        if(depth_mat)
        {
            cv::flip(undistortedMat,mirror,1);
            mirror /= 1000;
            cv::FileStorage file( dir +"/depth_mat/tmp-depth_mat.xml", cv::FileStorage::WRITE);
            file << "depth_mat" << mirror;
            file.release();
        }

        boost::filesystem::rename(dir +"/depth/tmp-depth.png",dir +"/depth/depth.png");
        boost::filesystem::rename(dir +"/rgb/tmp-rgb.png",dir +"/rgb/rgb.png");
        if(orig)
            boost::filesystem::rename(dir +"/orig_rgb/tmp-orig_rgb.jpg",dir +"/orig_rgb/orig_rgb.jpg");
        if(depth_mat)
            boost::filesystem::rename(dir +"/depth_mat/tmp-depth_mat.xml",dir +"/depth_mat/depth_mat.xml");

        if(store_all != "")
        {
            boost::filesystem::copy_file( dir +"/depth/depth.png",store_all +"/depth/"+time+".png");
            boost::filesystem::copy_file( dir +"/rgb/rgb.png",store_all +"/rgb/"+time+".png");
            if(orig)
                boost::filesystem::copy_file(dir +"/orig_rgb/orig_rgb.jpg",store_all +"/orig_rgb/"+time+".jpg");
            if(depth_mat)
                boost::filesystem::copy_file(dir +"/depth_mat/depth_mat.xml",store_all +"/depth_mat/"+time+".xml");
        }

        listener.release(frames);
    }
    dev->stop();
    dev->close();

    delete registration;

    return 0;
}
