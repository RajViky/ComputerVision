#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/core/core.hpp>

#include "pcd_helpers.h"
int main(int argc, char **argv)
{
    //Parameters parsing
    std::vector<std::string> args(argv, argv + argc);
    args.erase(args.begin());
    bool help = false;
    std::string path = "/tmp/kinect2/pcd";

    for(std::string arg : args)
    {
        if(arg.at(0) != '-' || arg == "help" || arg == "-help" || arg == "--help")
        {
            help = true;
            break;
        }
        arg.erase(arg.begin(),arg.begin()+1);
        if(arg.substr(0,4) == "path")
        {
            path = arg.substr(5,arg.length()-5);
            std::cout << "Path:   " << path << ";" << std::endl;
        }
    }
    if(help)
    {
        std::cout << "PCD Viewer" << std::endl;
        std::cout << "======================" << std::endl;
        std::cout << "Available Parameters:" << std::endl;
        std::cout << "-path         -path=/tmp/pcd" << std::endl;
        std::cout << "              -path=/tmp/file.pcd" << std::endl;
        std::cout << "     If path is file, only one file will be displayed." << std::endl;
        std::cout << "     If it is directory, all files in directory will be displayed." << std::endl;
        std::cout << "     Program can handle binary and text .pcd," << std::endl;
        std::cout << "     it will look for .trt and .trb files with the same name as .pcd," << std::endl;
        std::cout << "     these files should contain transformation matrix." << std::endl;
        std::cout << "     .trt (Transformation Text)" << std::endl;
        std::cout << "     .trb (Transformation Binary)" << std::endl;
        return 0;
    }

    boost::filesystem::path pathBoost = path;
    if ( exists( pathBoost ) )
    {
        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.setBackgroundColor (0, 0, 0);
        viewer.addCoordinateSystem (1.0);
        pcl::PCDReader reader;

        if(is_regular_file(pathBoost))
        {
            std::cout << pathBoost.stem();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            if(reader.read (pathBoost.string(), *cloud) == 0)
            {
                Eigen::Matrix4f t = PCD_Helpers::readTransform(pathBoost);
                if(t(0,0) != 0)
                {
                    std::cout << " has transformation";
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::transformPointCloud(*cloud,*tmp, t);
                    viewer.addPointCloud<pcl::PointXYZRGB> (tmp, pathBoost.stem().string());
                }
                else
                {
                    std::cout << " does not have transformation";
                    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, pathBoost.stem().string());
                }
                std::cout << std::endl;
            }
            else
            {
                std::cout << "Cannot read .pcd file." << std::endl;
                return -1;
            }
        }
        else if(is_directory(pathBoost))
        {
            boost::filesystem::directory_iterator end_itr;
            for (boost::filesystem::directory_iterator itr(pathBoost); itr != end_itr; ++itr)
            {
                if (is_regular_file(itr->path())) {
                    if(itr->path().extension().string() == ".pcd")
                    {
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                        if(reader.read (itr->path().string(), *cloud) == 0)
                        {
                            std::cout << itr->path().stem();

                            Eigen::Matrix4f t = PCD_Helpers::readTransform(itr->path());
                            if(t(0,0) != 0)
                            {
                                std::cout << " has transformation";
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
                                pcl::transformPointCloud(*cloud,*tmp, t);
                                viewer.addPointCloud<pcl::PointXYZRGB> (tmp, itr->path().stem().string());
                            }
                            else
                            {
                                std::cout << " does not have transformation";
                                viewer.addPointCloud<pcl::PointXYZRGB> (cloud, itr->path().stem().string());
                            }
                            std::cout << std::endl;
                        }
                        else
                        {
                            std::cout << "Cannot read .pcd file -> skip.";
                        }
                    }
                }
            }
        }
        viewer.spin();
        while(true)
        {
            usleep(100);
        }
    }
    else
    {
        cout << "Given path does not exists." << endl;
        return -1;
    }

    cout << "Hmm..." << endl;
    return -1;
}
