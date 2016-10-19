#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    Mat imageDepth ( 480, 640, CV_16UC1 );
    Mat imageRGB;

    // Video stream settings
    VideoCapture capture;
    capture.open( CAP_OPENNI2 );

//    if ( !capture.isOpened() ) {
//      cerr << "Cannot get video stream!" << endl;
//      exit ( 1 );
//    }

//    if ( !capture.grab() ) {
//      cerr << "Cannot grab images!" << endl;
//      exit ( 1 );
//    }

//    // Getting frames
//    if ( capture.retrieve( imageDepth, CAP_OPENNI_DEPTH_MAP ) ) {
//      imwrite( "/tmp/test1.png", imageDepth );
//    }
//    if( capture.retrieve( imageRGB, CAP_OPENNI_BGR_IMAGE ) ) {
//      imwrite("/tmp/test2.png" , imageRGB );
//    }

    return 0;

}
