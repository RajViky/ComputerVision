#ifndef PCD_HELPERS_H
#define PCD_HELPERS_H

#include <Eigen/Eigenvalues>
#include <boost/filesystem.hpp>

class PCD_Helpers
{
public:    
    static Eigen::Matrix4f readTransform(boost::filesystem::path pcdPath);
};

#endif // PCD_HELPERS_H
