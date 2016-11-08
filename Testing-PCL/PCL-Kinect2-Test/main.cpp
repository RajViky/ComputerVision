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
