#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/filesystem.hpp>

#include "../PCD-Viewer/pcd_helpers.h"

using namespace std;
int main(int argc, char **argv)
{
    //Parameters parsing
    std::vector<std::string> args(argv, argv + argc);
    args.erase(args.begin());
    bool help = false;
    std::string in = "/tmp/kinect2/pcd";
    std::string out = "/tmp/kinect2/pcd-out";
    std::string merged = "/tmp/kinect2/pcd-out.pcd"; //TODO: set this to ""
    float grid = 0.01;

    for(std::string arg : args)
    {
        if(arg.at(0) != '-' || arg == "help" || arg == "-help" || arg == "--help")
        {
            help = true;
            break;
        }
        arg.erase(arg.begin(),arg.begin()+1);
        if(arg.substr(0,2) == "in")
        {
            in = arg.substr(3,arg.length()-3);
            std::cout << "In:   " << in << ";" << std::endl;
        }
        else if(arg.substr(0,3) == "out")
        {
            out = arg.substr(4,arg.length()-4);
            std::cout << "Out:  " << out << ";" << std::endl;
        }
        else if(arg.substr(0,6) == "merged")
        {
            merged = arg.substr(7,arg.length()-7);
            std::cout << "Merged:  " << merged << ";" << std::endl;
        }
        else if(arg.substr(0,4) == "grid")
        {
            grid = (float)atof(arg.substr(5,arg.length()-5).c_str());
            if(grid <= 0)
            {
                std::cout << "Illegal voxel gird size, set to default."  << std::endl;
                grid = 0.01;
            }
            std::cout << "Voxel grid size:  " << grid << ";" << std::endl;
        }
    }
    if(help)
    {
        std::cout << "PCD Viewer" << std::endl;
        std::cout << "======================" << std::endl;
        std::cout << "Available Parameters:" << std::endl;
        std::cout << "-in           -in=/tmp/pcd" << std::endl;
        std::cout << "              -in=/tmp/file.pcd" << std::endl;
        std::cout << "     If path is file, only one file will be tranfromed." << std::endl;
        std::cout << "     If it is directory, all files in directory will be displayed." << std::endl;
        std::cout << "     Program can handle binary and text .pcd," << std::endl;
        std::cout << "     it will look for .trt and .trb files with the same name as .pcd," << std::endl;
        std::cout << "     these files should contain transformation matrix." << std::endl;
        std::cout << "     .trt (Transformation Text)" << std::endl;
        std::cout << "     .trb (Transformation Binary)" << std::endl;
        std::cout << "-out          -out=/tmp/out-pcd" << std::endl;
        std::cout << "              Output is treated as directory." << std::endl;
        std::cout << "              It will be CREATED if it does not exists." << std::endl;
        std::cout << "              It will be ERASED if it does exists." << std::endl;
        std::cout << "-merged       -merged=/tmp/out-pcd.pcd"<< std::endl;
        std::cout << "              If set all files will be merged to this file." << std::endl;
        std::cout << "              Disabled for single file." << std::endl;
        std::cout << "-grid         -grid=0.001"<< std::endl;
        std::cout << "              Voxel grid size for volxe filter on merged point cloud." << std::endl;
        std::cout << "              Float in meters. Default 0.01" << std::endl;
        return 0;
    }

    boost::filesystem::path pathIn = in;
    boost::filesystem::path pathOut = out;
    if ( exists( pathOut ) )
    {
        boost::filesystem::remove_all(pathOut);
    }
    if (!boost::filesystem::create_directories(pathOut))
    {
        std::cout << "Cannot create output file directory." << std::endl;
    }

    boost::filesystem::path pathMerged = merged;
    if(!exists(pathMerged.parent_path()) && !boost::filesystem::create_directories(pathMerged.parent_path()))
    {
        std::cout << "Cannot create parent directory for merged file." << std::endl;
    }
    if ( exists( pathIn ) )
    {
        pcl::PCDReader reader;

        if(is_regular_file(pathIn))
        {
            std::cout << pathIn.stem();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            if(reader.read (pathIn.string(), *cloud) == 0)
            {
                pcl::PCLPointCloud2 header;
                reader.readHeader(pathIn.string(), header);

                Eigen::Matrix4f t = PCD_Helpers::readTransform(pathIn);
                if(t(0,0) != 0)
                {
                    std::cout << " has transformation";
                    bool binary = false;
                    if(exists(pathIn.parent_path() / (pathIn.stem().string() + ".trb")))
                        binary = true;

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::transformPointCloud(*cloud,*tmp, t);

                    pcl::PCDWriter writer;
                    writer.write( (pathOut / pathIn.filename()).string(), *tmp, binary);
                }
                else
                {
                    std::cout << " does not have transformation - copy";
                    boost::filesystem::copy(pathIn.string(),(pathOut / pathIn.filename()).string());
                }
                std::cout << std::endl;
            }
            else
            {
                std::cout << "Cannot read .pcd file." << std::endl;
                return -1;
            }
        }
        else if(is_directory(pathIn))
        {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr wholeCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            boost::filesystem::directory_iterator end_itr;
            for (boost::filesystem::directory_iterator itr(pathIn); itr != end_itr; ++itr)
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
                                bool binary = false;
                                if(exists(itr->path().parent_path() / (itr->path().stem().string() + ".trb")))
                                    binary = true;

                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
                                pcl::transformPointCloud(*cloud,*tmp, t);

                                pcl::PCDWriter writer;
                                writer.write( (pathOut / itr->path().filename()).string(), *tmp, binary);
                                if(merged != "")
                                    *wholeCloud += *tmp;
                            }
                            else
                            {
                                std::cout << " does not have transformation - copy";
                                boost::filesystem::copy(itr->path().string(),(pathOut / itr->path().filename()).string());
                                if(merged != "")
                                    *wholeCloud += *cloud;
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


            if(wholeCloud->size())
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::VoxelGrid<pcl::PointXYZRGB> sor;
                sor.setInputCloud (wholeCloud);
                sor.setLeafSize (grid, grid, grid);
                sor.filter (*cloud_filtered);

                pcl::PCDWriter writer;
                writer.write( merged, *cloud_filtered, true);
                return 0;
            }
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

