#ifndef VISION_DATA_HPP_INCLUDED
#define VISION_DATA_HPP_INCLUDED

#include <Eigen/Core>
#include <Eigen/SVD>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "findtoolontable/vision_msgs.h"
#include "findtoolontable/pcl_vector.h"

struct vision_each    //each vision data
{
        float objhight2table;
        pcl::PointCloud<pcl::PointXYZ> hullClustersOnAxis;
        Eigen::Matrix4d pro2tableTFs;
        std::vector<pcl::PointCloud<pcl::PointXYZ> > toolConvexHulls;
};

//vision data
struct vision_data
{
        std::vector< float > objhight2table;
        std::vector< pcl::PointCloud<pcl::PointXYZ> > hullClustersOnAxis;
        std::vector< Eigen::Matrix4d> pro2tableTFs;
        std::vector< std::vector<pcl::PointCloud<pcl::PointXYZ> > > toolConvexHulls;
};

#endif