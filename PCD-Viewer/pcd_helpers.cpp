#include "pcd_helpers.h"
#include <fstream>

Eigen::Matrix4f PCD_Helpers::readTransform(boost::filesystem::path pcdPath) {
    std::vector<float>nL;
    if(exists(pcdPath.parent_path() / (pcdPath.stem().string() + ".trb")))
    {
        std::ifstream inFile;

        inFile.open((pcdPath.parent_path() / (pcdPath.stem().string() + ".trb")).string(),std::ios::in | std::ios::binary);
        float f;
        while( inFile.read(reinterpret_cast<char *>(&f), sizeof(f)))
            nL.push_back(f);
        inFile.close();
    }
    else if(exists(pcdPath.parent_path() / (pcdPath.stem().string() + ".trt")))
    {
        std::ifstream inFile;
        inFile.open((pcdPath.parent_path() / (pcdPath.stem().string() + ".trt")).string());

        double number = 0 ;
        while(inFile >> number)
        {
            nL.push_back(number);
        }
        inFile.close();
    }
    if(nL.size() == 16)
    {
        Eigen::Matrix4f ret;
        for(int i = 0;i<16;i++)
        {
            ret(i/4, i%4) = nL.at(i);
        }
        return ret;
    }
    Eigen::Matrix4f ret;
    ret.setZero(4,4);
    return ret;
}
