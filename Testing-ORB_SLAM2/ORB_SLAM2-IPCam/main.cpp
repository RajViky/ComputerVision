/**
* Based on examples from ORB-SLAM2 (Copyright (C) 2014-2016 Raúl Mur-Artal).
*
* For more information see <https://github.com/raulmur/ORB_SLAM2>*
*/
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

int main( int argc, char *argv[] )
{
    std::vector<std::string> arguments(argv + 1, argv + argc);

    //Vocabulary for detecting objects
    string Vocabulary = "/home/beda/data/skola/_Oulu/ORB_SLAM2/Vocabulary/ORBvoc.txt";
    //ConfigFile
    //string Yaml = "/home/beda/data/skola/_Oulu/ComputerVision/calibration//Honor3C-1280x720.yaml";
    string Yaml = "/home/beda/data/skola/_Oulu/ComputerVision/calibration//Honor3C-640x480.yaml";
    //Video source
    cv::VideoCapture cap;
    std::string videoStreamAddress = "http://192.168.0.102:8080/video";

    if(arguments.size() != 3)
    {
        std::cout  << "" << std::endl;
        std::cout  << "Usage: ORB_SLAM2-webcam VOCABULARY CONFIG.yaml VIDEO_STREAM" << std::endl;
        std::cout  << "Using default values" << std::endl;
        std::cout  << "" << std::endl;
    }
    else
    {
        Vocabulary = arguments[0];
        Yaml = arguments[1];
        videoStreamAddress = arguments[2];
    }


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(Vocabulary,Yaml,ORB_SLAM2::System::MONOCULAR,true);

    if(!cap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    std::cout << endl << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;

    // Main loop
    cv::Mat im;
    //for(int ni=0; ni<nImages; ni++)
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while(true)
    {
        //Read image from camera
        cap >> im;

        if(im.empty())
        {
            std::cerr << endl << "Failed to load image" << std::endl;
            return 1;
        }
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        double t = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - begin).count();
        SLAM.TrackMonocular(im,t);
    }

    // Stop all threads
    SLAM.Shutdown();


//    // Save camera trajectory
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
