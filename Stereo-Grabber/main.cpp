#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>

bool shutdown = false;
void sigint_handler(int)
{
    shutdown = true;
}
int main(int argc, char **argv)
{
    int id_left = 1;
    int id_right= 2;
    std::string dir = "";
    float freq = 30;
    size_t framemax = -1;
    float scale = 1;
    int numberOfDisparities = 0;// The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16
    int SADWindowSize = 0;//The block size (--blocksize=<...>) must be a positive odd number
    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
    int alg = STEREO_SGBM;
    if(dir == "")
        dir = "/tmp/stereo";

    std::string store_all = "";
    //store_all = dir + "/all";

    try
    {
        boost::filesystem::path path = dir;
        if ( exists( path ) )
        {
            boost::filesystem::remove_all(path);
        }
        if (!boost::filesystem::create_directories(path / "left")
                || !boost::filesystem::create_directories(path / "right")
                || !boost::filesystem::create_directories(path / "depth")
                || !boost::filesystem::create_directories(path / "left-undistort")
                || !boost::filesystem::create_directories(path / "right-undistort"))
        {
            std::cout << "Cannot create tmp directories" << std::endl;
            return -1;
        }

        if(store_all != "")
        {
            path = store_all;
            if ((!boost::filesystem::is_directory(path / "left") && !boost::filesystem::create_directories(path / "left"))
                    || (!boost::filesystem::is_directory(path / "right") && !boost::filesystem::create_directories(path / "right"))
                    || (!boost::filesystem::is_directory(path / "depth") && !boost::filesystem::create_directories(path / "depth"))
                    || (!boost::filesystem::is_directory(path / "left-undistort") && !boost::filesystem::create_directories(path / "left-undistort"))
                    || (!boost::filesystem::is_directory(path / "right-undistort") && !boost::filesystem::create_directories(path / "right-undistort")))
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

    cv::VideoCapture cap_left(id_left);
    cv::VideoCapture cap_right(id_right);

    cap_left.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap_left.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap_right.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap_right.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    boost::posix_time::ptime last_image = boost::posix_time::microsec_clock::local_time();

    if(!cap_left.isOpened() || !cap_right.isOpened())
        return -1;

    size_t framecount = 0;


    cv::Mat left;
    cv::Mat right;
    cap_left >> left;
    cap_right >> right;
    if(left.size != right.size)
    {
        std::cout << "Image size mismatch " << std::endl;
        return -1;
    }
    cv::Size img_size = left.size();

    cv::Mat M1, D1, M2, D2;
    std::string filePath = "/home/beda/data/skola/_Oulu/ComputerVision/build-StereoCalib-Desktop-Vydání/intrinsics.yml";
    boost::filesystem::path p = filePath;
    p = filePath;
    if(exists(p))
    {
        cv::FileStorage fs(filePath, cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            std::cout << "Failed to open file " << filePath << std::endl;
            return -1;
        }
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;
        M1 *= scale;
        M2 *= scale;
    }

    cv::Rect roi1, roi2;
    cv::Mat Q;

    cv::Mat R, T, R1, P1, R2, P2;
    filePath = "/home/beda/data/skola/_Oulu/ComputerVision/build-StereoCalib-Desktop-Vydání/extrinsics.yml";
    p = filePath;
    if(exists(p))
    {
        cv::FileStorage fs(filePath, cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            std::cout << "Failed to open file " << filePath << std::endl;
            return -1;
        }
        fs["R"] >> R;
        fs["T"] >> T;

        cv::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
    }

    while(!shutdown && framecount < framemax)
    {
        cv::Mat left;
        cv::Mat right;
        cap_left >> left;
        cap_right >> right;

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

        cv::imwrite(dir+"/left/tmp-left.png",left);
        cv::imwrite(dir+"/right/tmp-right.png",right);

        cv::Mat map11, map12, map21, map22;
        cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        cv::Mat img1r, img2r;
        cv::remap(left, img1r, map11, map12, cv::INTER_LINEAR);
        cv::remap(right, img2r, map21, map22, cv::INTER_LINEAR);

        cv::imwrite(dir+"/left-undistort/tmp-left.png",img1r);
        cv::imwrite(dir+"/right-undistort/tmp-right.png",img2r);

        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

        cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16,9);
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);

        bm->setROI1(roi1);
        bm->setROI2(roi2);
        bm->setPreFilterCap(31);
        bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
        bm->setMinDisparity(0);
        bm->setNumDisparities(numberOfDisparities);
        bm->setTextureThreshold(10);
        bm->setUniquenessRatio(15);
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(32);
        bm->setDisp12MaxDiff(1);

        sgbm->setPreFilterCap(63);
        int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
        sgbm->setBlockSize(sgbmWinSize);

        int cn = img1r.channels();

        sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setMode(alg == STEREO_HH ? cv::StereoSGBM::MODE_HH : cv::StereoSGBM::MODE_SGBM);

        cv::Mat disp, disp8, disp16;
        //Mat img1p, img2p, dispp;
        //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
        //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

        //int64 t = getTickCount();
        if( alg == STEREO_BM )
            bm->compute(img1r, img2r, disp);
        else if( alg == STEREO_SGBM || alg == STEREO_HH )
            sgbm->compute(img1r, img2r, disp);
        //t = getTickCount() - t;
        //printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

        //disp = dispp.colRange(numberOfDisparities, img1p.cols);
        disp.convertTo(disp16, CV_16U);

        if( alg != STEREO_VAR )
            disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        else
            disp.convertTo(disp8, CV_8U);

        cv::imwrite(dir+"/depth/tmp-depth.png",disp8);
        //cv::imwrite(dir+"/depth/tmp-depth.png",disp8);

//        fflush(stdout);
//        cv::Mat xyz;
//        cv::reprojectImageTo3D(disp, xyz, Q, true);
//        const double max_z = 1.0e4;
//        FILE* fp = fopen((dir+"/tmp-pcd.pcd").c_str(), "wt");
//        for(int y = 0; y < xyz.rows; y++)
//        {
//            for(int x = 0; x < xyz.cols; x++)
//            {
//                cv::Vec3f point = xyz.at<cv::Vec3f>(y, x);
//                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
//                fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
//            }
//        }
//        fclose(fp);
//        boost::filesystem::rename(dir+"/tmp-pcd.pcd",dir+"/pcd.pcd");


        boost::filesystem::rename(dir+"/left/tmp-left.png",dir+"/left/left.png");
        boost::filesystem::rename(dir+"/right/tmp-right.png",dir+"/right/right.png");
        boost::filesystem::rename(dir+"/left-undistort/tmp-left.png",dir+"/left-undistort/left.png");
        boost::filesystem::rename(dir+"/right-undistort/tmp-right.png",dir+"/right-undistort/right.png");
        boost::filesystem::rename(dir+"/depth/tmp-depth.png",dir+"/depth/depth.png");

        if(store_all != "")
        {
            boost::filesystem::copy_file(dir+"/left/left.png",store_all +"/left/"+time+".png");
            boost::filesystem::copy_file(dir+"/right/right.png",store_all +"/right/"+time+".png");
            boost::filesystem::copy_file(dir+"/left-undistort/left.png",store_all +"/left-undistort/"+time+".png");
            boost::filesystem::copy_file(dir+"/right-undistort/right.png",store_all +"/right-undistort/"+time+".png");
            boost::filesystem::copy_file(dir+"/depth/depth.png",store_all +"/depth/"+time+".png");
        }

        framecount++;
    }

    std::cout << "Hello World!" << std::endl;
    return 0;
}
