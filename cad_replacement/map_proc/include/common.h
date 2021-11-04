#ifndef COMMON_H_
#define COMMON_H_

#include <iostream>
#include <vector>
#include <cmath>  
#include <sys/stat.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

// include pcl header first to avoid building error
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace MapProcessing
{
    
using PointT = pcl::PointXYZ;
using PointTFull = pcl::PointXYZRGBNormal;
using PointTLabel = pcl::PointXYZRGBL;

struct OBBox{
    Eigen::Vector3f pos; // x,y,z in world frame
    Eigen::Vector3f aligned_dims; 
    Eigen::Quaternionf quat; // x,y,z,w
};

}


#endif