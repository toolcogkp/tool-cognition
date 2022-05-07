/**
 * @file vision_data.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * header for vision data received from perception module
 * @version 0.1
 * @date 2019-03-24
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef VISION_DATA_HPP_INCLUDED
#define VISION_DATA_HPP_INCLUDED

// eigen
#include <Eigen/Core>
#include <Eigen/SVD>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// custom msgs from vision_ws
#include "findtoolontable/vision_msgs.h"
#include "findtoolontable/pcl_vector.h"

/**
 * @brief vision data structure for each perceived object
 * 
 */
struct vision_each    //each vision data
{
        float objhight2table;
        pcl::PointCloud<pcl::PointXYZ> hullClustersOnAxis;
        Eigen::Matrix4d pro2tableTFs;
        std::vector<pcl::PointCloud<pcl::PointXYZ> > toolConvexHulls;
};

//vision data
/**
 * @brief vision data structure for large sets of perceived objects
 * 
 */
struct vision_data
{
        std::vector< float > objhight2table;
        std::vector< pcl::PointCloud<pcl::PointXYZ> > hullClustersOnAxis;
        std::vector< Eigen::Matrix4d> pro2tableTFs;
        std::vector< std::vector<pcl::PointCloud<pcl::PointXYZ> > > toolConvexHulls;
};

#endif