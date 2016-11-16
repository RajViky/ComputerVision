#include <sstream>
#include <signal.h>

#include <librealsense/rs.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>

bool shutdown = false;
void sigint_handler(int)
{
    shutdown = true;
}
int main(int argc, char **argv) try
{
    std::vector<std::string> args(argv, argv + argc);
    args.erase(args.begin());
    bool help = false;
    std::string dir = "";
    std::string store_all = "";
    //bool orig = false;
    float freq = 0;
    size_t framemax = -1;
    int device_id = 0;

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
        else if(arg.substr(0,2) == "id")
        {
            device_id = std::stoi(arg.substr(3,arg.length()-3));
            std::cout << "Device ID:   " << device_id << "Hz;" << std::endl;
        }
        else if(arg.substr(0,1) == "f")
        {
            freq = std::stof(arg.substr(2,arg.length()-2));
            std::cout << "Frequency:   " << freq << "Hz;" << std::endl;
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
        std::cout << "Intel RealSense image grabber" << std::endl;
        std::cout << "======================" << std::endl;
        std::cout << "Program should be closed with Ctrl+C" << std::endl;
        std::cout << "Available Parameters:" << std::endl;
        std::cout << "-id          -id=1" << std::endl;
        std::cout << "     ID of device, default 0." << std::endl;
        std::cout << "-dir          -dir=/tmp/grabber" << std::endl;
        std::cout << "     Base tmp directory where data is stored." << std::endl;
        std::cout << "     Path WILL BE CLEARED if it does exists." << std::endl;
        std::cout << "     Path WILL BE CREATED if it does not exists." << std::endl;
        std::cout << "     If not set /tmp/kinect2 is used." << std::endl;
        std::cout << "-f            -f=60" << std::endl;
        std::cout << "     Capture frequency in Hz." << std::endl;
        std::cout << "     If not set or set to 0, data is stored as fast as possible." << std::endl;
        std::cout << "-store_all    -store_all=/path/data" << std::endl;
        std::cout << "     If set: all images will be stored with timestamps in filenames to path" << std::endl;
        std::cout << "     Path WILL NOT BE CLEARED if it does exists." << std::endl;
        std::cout << "     Path WILL BE CREATED if it does not exists." << std::endl;
        std::cout << "     If not set: only current images will be stored." << std::endl;
        std::cout << "-max_frames   -max_frames=1000" << std::endl;
        std::cout << "     Limits maximum number of recorded frames." << std::endl;
        std::cout << "     Applicatin will exit when limit is reached." << std::endl;
        std::cout << "     Unlimited if not set." << std::endl;
        std::cout << " --------------------------------------" << std::endl;
        //        std::cout << " " << std::endl;
        //        std::cout << "-orig" << std::endl;
        //        std::cout << "     RGB images in original resolution are also stored." << std::endl;
        //        std::cout << " --------------------------------------" << std::endl;
        std::cout << " -clean" << std::endl;
        std::cout << "     Run clenup after unpropper exit." << std::endl;
        std::cout << " -help" << std::endl;
        std::cout << "     Print this help." << std::endl;
        return 0;
    }

    if(dir == "")
        dir = "/tmp/realsense";

    try
    {
        boost::filesystem::path path = dir;
        if ( exists( path ) )
        {
            boost::filesystem::remove_all(path);
        }
        if (!boost::filesystem::create_directories(path / "rgb")
                || !boost::filesystem::create_directories(path / "depth"))
        {
            std::cout << "Cannot create tmp directories" << std::endl;
            return -1;
        }

        if(store_all != "")
        {
            path = store_all;
            if ((!boost::filesystem::is_directory(path / "rgb") && !boost::filesystem::create_directories(path / "rgb"))
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

    //^C
    signal(SIGINT,sigint_handler);
    //abort()
    signal(SIGABRT, sigint_handler);
    //sent by "kill" command
    signal(SIGTERM, sigint_handler);
    //^Z
    signal(SIGTSTP, sigint_handler);

    size_t framecount = 0;

    boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    boost::posix_time::ptime last_image = boost::posix_time::microsec_clock::local_time();

    //==========
    //==========
    //REAL SENSE
    //==========
    //==========

    rs::context ctx;
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    int device_count = ctx.get_device_count();
    if (!device_count) printf("No device detected. Is it plugged in?\n");
    for(int i = 0; i < device_count; ++i)
    {
        // Show the device name and information
        rs::device * dev = ctx.get_device(i);
        std::cout << "Device " << i << " - " << dev->get_name() << ":\n";
        std::cout << " Serial number: " << dev->get_serial() << "\n";
        std::cout << " Firmware version: " << dev->get_firmware_version() << "\n";
        try { std::cout << " USB Port ID: " << dev->get_usb_port_id() << "\n"; } catch (...) {}
        if (dev->supports(rs::capabilities::adapter_board)) std::cout << " Adapter Board Firmware version: " << dev->get_info(rs::camera_info::adapter_board_firmware_version) << "\n";
        if (dev->supports(rs::capabilities::motion_events)) std::cout << " Motion Module Firmware version: " << dev->get_info(rs::camera_info::motion_module_firmware_version) << "\n";
        // Show which options are supported by this device
        std::cout << " Supported options:\n";
        for(int j = 0; j < RS_OPTION_COUNT; ++j)
        {
            rs::option opt = (rs::option)j;
            if(dev->supports_option(opt))
            {
                double min, max, step, def;
                dev->get_option_range(opt, min, max, step, def);
                std::cout << "    " << opt << " : " << min << " .. " << max << ", " << step << ", " << def << "\n";
            }
        }

        // Show which streams are supported by this device
        for(int j = 0; j < RS_STREAM_COUNT; ++j)
        {
            // Determine number of available streaming modes (zero means stream is unavailable)
            rs::stream strm = (rs::stream)j;
            int mode_count = dev->get_stream_mode_count(strm);
            if(mode_count == 0) continue;

            // Show each available mode for this stream
            std::cout << " Stream " << strm << " - " << mode_count << " modes:\n";
            for(int k = 0; k < mode_count; ++k)
            {
                // Show width, height, format, and framerate, the settings required to enable the stream in this mode
                int width, height, framerate;
                rs::format format;
                dev->get_stream_mode(strm, k, width, height, format, framerate);
                std::cout << "  " << width << "\tx " << height << "\t@ " << framerate << "Hz\t" << format;

                // Enable the stream in this mode so that we can retrieve its intrinsics
                dev->enable_stream(strm, width, height, format, framerate);
                rs::intrinsics intrin = dev->get_stream_intrinsics(strm);

                // Show horizontal and vertical field of view, in degrees
                std::cout << "\t" << std::setprecision(3) << intrin.hfov() << " x " << intrin.vfov() << " degrees\n";
            }

            // Some stream mode combinations are invalid, so disable this stream before moving on to the next one
            dev->disable_stream(strm);
        }
    }

    rs::device & dev = *ctx.get_device(device_id);

    //CREATE CONFIGS
    std::string calibration_path = dir;
    std::string calibration_name = "default";
    std::string name = dev.get_name();
    if(name == "Intel RealSense R200")
    {
        std::ofstream calibration_file;
        calibration_file.open (calibration_path+"/"+calibration_name+".yaml");
        calibration_file << "%YAML:1.0" << std::endl;
        calibration_file << "camera_name: R200" << std::endl;
        calibration_file << "image_width: 640" << std::endl;
        calibration_file << "image_height: 480" << std::endl;
        calibration_file << "camera_matrix:" << std::endl;
        calibration_file << "   rows: 3" << std::endl;
        calibration_file << "   cols: 3" << std::endl;
        calibration_file << "   data: [ 6.2944019856577290e+02, 0., 3.0622489799542683e+02, 0.," << std::endl;
        calibration_file << "       6.3477664417957772e+02, 2.5183992953451113e+02, 0., 0., 1. ]" << std::endl;
        calibration_file << "distortion_coefficients:" << std::endl;
        calibration_file << "   rows: 1" << std::endl;
        calibration_file << "   cols: 5" << std::endl;
        calibration_file << "   data: [ 2.3002469618070301e-01, -1.5154600483631122e+00," << std::endl;
        calibration_file << "       2.9248724196160461e-03, 1.0826683306159588e-03," << std::endl;
        calibration_file << "       2.2270009222579970e+00 ]" << std::endl;
        calibration_file << "distortion_model: plumb_bob" << std::endl;
        calibration_file << "rectification_matrix:" << std::endl;
        calibration_file << "   rows: 3" << std::endl;
        calibration_file << "   cols: 3" << std::endl;
        calibration_file << "   data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]" << std::endl;
        calibration_file << "projection_matrix:" << std::endl;
        calibration_file << "   rows: 3" << std::endl;
        calibration_file << "   cols: 4" << std::endl;
        calibration_file << "   data: [ 6.2944019856577290e+02, 0., 3.0622489799542683e+02, 0., 0.," << std::endl;
        calibration_file << "       6.3477664417957772e+02, 2.5183992953451113e+02, 0., 0., 0., 1.," << std::endl;
        calibration_file << "       1. ]" << std::endl;
        calibration_file.close();
    }
    //dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    //dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    dev.enable_stream(rs::stream::depth, 640,480,rs::format::z16, 60);
    dev.enable_stream(rs::stream::color, 640,480,rs::format::rgb8, 60);
    dev.start();

    while (!shutdown && (framemax == (size_t)-1 || framecount < framemax))
    {
        dev.wait_for_frames();

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


        cv::Mat rgb(480, 640, CV_8UC3, (uchar *) dev.get_frame_data(rs::stream::color));
        cv::Mat depth16(480, 640, CV_16U, (uchar *) dev.get_frame_data(rs::stream::depth_aligned_to_color));
        //cv::Mat rgb_orig(360, 480, CV_8UC3, (uchar *) dev.get_frame_data(rs::stream::color_aligned_to_depth));
        //        cv::Mat orig_rgb(480, 640, CV_8UC3, (uchar *) dev.get_frame_data(rs::stream::color_aligned_to_depth));
        //        cv::Mat orig_depth(480, 640, CV_16U, (uchar *) dev.get_frame_data(rs::stream::depth));

        imwrite( dir +"/depth/tmp-depth.png", depth16 );
        imwrite( dir +"/rgb/tmp-rgb.png", rgb );
        //        if(orig)
        //        {
        //            imwrite( dir +"/orig_rgb/tmp-orig_rgb.jpg", orig_rgb );
        //            imwrite( dir +"/orig_rgb/tmp-orig_depth.jpg", orig_depth );
        //        }

        boost::filesystem::rename(dir +"/depth/tmp-depth.png",dir +"/depth/depth.png");
        boost::filesystem::rename(dir +"/rgb/tmp-rgb.png",dir +"/rgb/rgb.png");
        //        if(orig)
        //        {
        //            boost::filesystem::rename(dir +"/orig_rgb/tmp-orig_rgb.jpg",dir +"/orig_rgb/orig_rgb.jpg");
        //            boost::filesystem::rename(dir +"/orig_rgb/tmp-orig_depth.jpg",dir +"/orig_rgb/orig_depth.jpg");
        //        }

        if(store_all != "")
        {
            boost::filesystem::copy_file( dir +"/depth/depth.png",store_all +"/depth/"+time+".png");
            boost::filesystem::copy_file( dir +"/rgb/rgb.png",store_all +"/rgb/"+time+".png");
            //            if(orig)
            //                boost::filesystem::copy_file(dir +"/orig_rgb/orig_rgb.jpg",store_all +"/orig_rgb/"+time+".jpg");
        }
    }
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
