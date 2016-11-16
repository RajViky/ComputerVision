#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

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

    if(dir == "")
        dir = "/tmp/stereo";


    std::string store_all = dir + "/all";

    try
    {
        boost::filesystem::path path = dir;
        if ( exists( path ) )
        {
            boost::filesystem::remove_all(path);
        }
        if (!boost::filesystem::create_directories(path / "left")
                || !boost::filesystem::create_directories(path / "right")
                || !boost::filesystem::create_directories(path / "depth"))
        {
            std::cout << "Cannot create tmp directories" << std::endl;
            return -1;
        }

        if(store_all != "")
        {
            path = store_all;
            if ((!boost::filesystem::is_directory(path / "left") && !boost::filesystem::create_directories(path / "left"))
                    || (!boost::filesystem::is_directory(path / "right") && !boost::filesystem::create_directories(path / "right"))
                    || (!boost::filesystem::is_directory(path / "depth") && !boost::filesystem::create_directories(path / "depth")))
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
    boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    boost::posix_time::ptime last_image = boost::posix_time::microsec_clock::local_time();

    if(!cap_left.isOpened() || !cap_right.isOpened())
        return -1;

    size_t framecount = 0;

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

        cv::Mat stereo = cv::StereoBM.create();
        cv::Mat disparity = stereo.compute(left,right);

        cv::imwrite(dir+"/left/tmp-left.png",left);
        cv::imwrite(dir+"/right/tmp-right.png",right);

        boost::filesystem::rename(dir+"/left/tmp-left.png",dir+"/left/left.png");
        boost::filesystem::rename(dir+"/right/tmp-right.png",dir+"/right/right.png");

        if(store_all != "")
        {
            boost::filesystem::copy_file(dir+"/left/left.png",store_all +"/left/"+time+".png");
            boost::filesystem::copy_file(dir+"/right/right.png",store_all +"/right/"+time+".png");
        }

        framecount++;
    }

    std::cout << "Hello World!" << std::endl;
    return 0;
}
