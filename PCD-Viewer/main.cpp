#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/util3d_transforms.h"


int main(int argc, char *argv[])
{
    std::string dir = "/tmp/kinect2/pcd";
    boost::filesystem::path path = dir;

    if ( exists( path ) )
    {
        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.setBackgroundColor (0, 0, 0);
        viewer.addCoordinateSystem (1.0);
        pcl::PCDReader reader;
        boost::filesystem::directory_iterator end_itr;
        for (boost::filesystem::directory_iterator itr(path); itr != end_itr; ++itr)
        {
            // If it's not a directory, list it. If you want to list directories too, just remove this check.
            if (is_regular_file(itr->path())) {
                // assign current file name to current_file and echo it out to the console.
                if(itr->path().extension().string() == ".pcd")
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                    if(reader.read (itr->path().string(), *cloud) == 0)
                    {
                        std::cout << itr->path().stem();

                        if(exists(itr->path().parent_path() / (itr->path().stem().string() + ".trt")) || exists(itr->path().parent_path() / (itr->path().stem().string() + ".trb")))
                        {
                            std::cout << " has transformation";
                            std::vector<float>nL;
                            if(exists(itr->path().parent_path() / (itr->path().stem().string() + ".trb")))
                            {
                                std::ifstream inFile;

                                inFile.open((itr->path().parent_path() / (itr->path().stem().string() + ".trb")).string(),std::ios::in | std::ios::binary);
                                float f;
                                while( inFile.read(reinterpret_cast<char *>(&f), sizeof(f)))
                                    nL.push_back(f);
                                inFile.close();
                            }
                            else if(exists(itr->path().parent_path() / (itr->path().stem().string() + ".trt")))
                            {
                                std::ifstream inFile;
                                inFile.open((itr->path().parent_path() / (itr->path().stem().string() + ".trt")).string());

                                double number = 0 ;
                                while(inFile >> number)
                                {
                                    nL.push_back(number);
                                }
                                inFile.close();
                            }

                            if(nL.size() != 12)
                            {
                                std::cout << std::endl << "Error reading transform file." << std::endl;
                                //return -1; //TODO: too strict, just for debugging
                            }
                            else
                            {
                                rtabmap::Transform t(nL[0],nL[1],nL[2],nL[3],nL[4],nL[5],nL[6],nL[7],nL[8],nL[9],nL[10],nL[11]);
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
                                *tmp += *rtabmap::util3d::transformPointCloud(cloud, t);
                                viewer.addPointCloud<pcl::PointXYZRGB> (tmp, itr->path().stem().string());
                            }
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

    cout << "Hello World!" << endl;
    return 0;
}
