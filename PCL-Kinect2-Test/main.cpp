#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <iostream>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
    PointCloud<PointXYZRGBA>::Ptr sourceCloud(new PointCloud<PointXYZRGBA>);

    boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr&)> function = [&sourceCloud](const PointCloud<PointXYZRGBA>::ConstPtr &cloud)
    {
        copyPointCloud(*cloud, *sourceCloud);
    };

    // Create Kinect2Grabber
    Grabber* grabber = new io::OpenNI2Grabber();
    // Regist Callback Function
    grabber->registerCallback(function);
    // Start Retrieve Data
    grabber->start();
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    // Stop Retrieve Data
    grabber->stop();
    cout << "The Cloud size: " << sourceCloud->size() << " points ..." << endl;
    cout << "One point = " << sourceCloud->points.at(1000) << "|" << endl;
}


////#define _GLIBCXX_USE_CXX11_ABI 0
//#include <Eigen/StdVector>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/openni2_grabber.h>


//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/voxel_grid.h>


////planar segmentation
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>

//#include <pcl/filters/extract_indices.h>

//class SimpleOpenNIViewer
//{
//public:
//    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

//    //void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
//    //    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
//    //    {
//    //        if (!viewer.wasStopped())
//    //        {
//    //            //Pass Through filter
//    //            //            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
//    //            //            pcl::PassThrough<pcl::PointXYZRGBA> pass;
//    //            //            pass.setInputCloud (cloud);
//    //            //            pass.setFilterFieldName("z");
//    //            //            pass.setFilterLimits(0.0, 2.0);
//    //            //            //pass.setFilterLimitsNegative (true);
//    //            //            pass.filter(*cloud_filtered);

//    //            //            viewer.showCloud(cloud_filtered);

//    //            //Voxel Grid filter
//    //            //            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
//    //            //            pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
//    //            //            sor.setInputCloud(cloud);
//    //            //            sor.setLeafSize(0.1f, 0.1f, 0.1f);
//    //            //            sor.filter(*cloud_filtered);
//    //            //            viewer.showCloud(cloud_filtered);


//    //            //Planar segmentation
//    //            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//    //            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//    //            // Create the segmentation object
//    //            pcl::SACSegmentation<pcl::PointXYZ> seg;
//    //            // Optional
//    //            seg.setOptimizeCoefficients(true);
//    //            // Mandatory
//    //            seg.setModelType(pcl::SACMODEL_PLANE);
//    //            seg.setMethodType(pcl::SAC_RANSAC);
//    //            seg.setDistanceThreshold(0.01);

//    //            seg.setInputCloud(cloud);
//    //            seg.segment(*inliers, *coefficients);

//    //            std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//    //                                                  << coefficients->values[1] << " "
//    //                                                  << coefficients->values[2] << " "
//    //                                                  << coefficients->values[3] << std::endl;

//    ////              std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
//    ////              for (size_t i = 0; i < inliers->indices.size (); ++i)
//    ////                std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
//    ////                                                           << cloud->points[inliers->indices[i]].y << " "
//    ////                                                           << cloud->points[inliers->indices[i]].z << std::endl;

//    //            viewer.showCloud(cloud);
//    //        }
//    //    }

//    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
//    {
//        if (!viewer.wasStopped())
//        {
////            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);



////            // Create the filtering object: downsample the dataset using a leaf size of 1cm
////            pcl::VoxelGrid<pcl::PointXYZ> sor;
////            sor.setInputCloud(cloud);
////            sor.setLeafSize(0.01f, 0.01f, 0.01f);
////            sor.filter(*cloud_filtered);


////            //std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

////            // Write the downsampled version to disk
////            pcl::PCDWriter writer;
////            writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

////            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
////            pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
////            // Create the segmentation object
////            pcl::SACSegmentation<pcl::PointXYZ> seg;
////            // Optional
////            seg.setOptimizeCoefficients(true);
////            // Mandatory
////            seg.setModelType(pcl::SACMODEL_PLANE);
////            seg.setMethodType(pcl::SAC_RANSAC);
////            seg.setMaxIterations(1000);
////            seg.setDistanceThreshold(0.1);

////            // Create the filtering object
////            pcl::ExtractIndices<pcl::PointXYZ> extract;

////            int i = 0, nr_points = (int) cloud_filtered->points.size();
////            // While 30% of the original cloud is still there
////            while (cloud_filtered->points.size() > 0.3 * nr_points)
////            {
////                // Segment the largest planar component from the remaining cloud
////                seg.setInputCloud(cloud_filtered);
////                seg.segment(*inliers, *coefficients);
////                if (inliers->indices.size() == 0)
////                {
////                    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
////                    break;
////                }

////                // Extract the inliers
////                extract.setInputCloud(cloud_filtered);
////                extract.setIndices(inliers);
////                extract.setNegative(false);
////                extract.filter(*cloud_p);
////                std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

////                std::stringstream ss;
////                ss << "table_scene_lms400_plane_" << i << ".pcd";
////                writer.write<pcl::PointXYZ> (ss.str(), *cloud_p, false);

////                // Create the filtering object
////                extract.setNegative(true);
////                extract.filter(*cloud_f);
////                cloud_filtered.swap(cloud_f);
////                i++;
////            }

//            viewer.showCloud(cloud);
//        }
//    }

//    void run ()
//    {
//        pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

//        //boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
//        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

//        interface->registerCallback (f);

//        interface->start ();

//        while (!viewer.wasStopped())
//        {
//            boost::this_thread::sleep (boost::posix_time::seconds (1));
//        }

//        interface->stop ();
//    }

//    pcl::visualization::CloudViewer viewer;
//};

//int main ()
//{
//    SimpleOpenNIViewer v;
//    v.run ();
//    return 0;
//}
