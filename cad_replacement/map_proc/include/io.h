#ifndef IO_H_
#define IO_H_

#include <errno.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>

#include "common.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pg_map_ros/nodes.h>
#include <pg_map_ros/parse_graph_map.h>

namespace MapProcessing
{

// Creates the given path, unless the path already exists, in which case
// nothing is done and 0 (success) is returned.
inline int makePath(const std::string& path_in, mode_t mode) {
  size_t previous_slash_pos = 0u;
  size_t current_slash_pos;
  std::string dir;
  int make_dir_status = 0;

  std::string path = path_in;
  if(!path.empty())
  {
    if (path.back() != '/') {
        // Add a trailing '/' so we can handle everything in loop.
        path += '/';
    }
  }

  while ((current_slash_pos = path.find_first_of('/', previous_slash_pos)) !=
         std::string::npos) {
    dir = path.substr(0, current_slash_pos++);
    previous_slash_pos = current_slash_pos;
    if (dir.empty()) continue;  // If leading / first time is 0 length.
    if (dir == ".") continue;

    const char* c_str = dir.c_str();
    for (size_t i = 0u; i < strlen(c_str); ++i) {
      if (c_str[i] < static_cast<char>(32) ||
          c_str[i] > static_cast<char>(126)) {
        return -1;
      }
    }

    if ((make_dir_status = mkdir(c_str, mode)) && errno != EEXIST) {
      return make_dir_status;
    }
  }
  // We return -1 on error above, so just return 0 here.
  return 0;
}


bool ifDirectoryExist(std::string filePath)
{
    try {
        // Create a Path object from given path string
        boost::filesystem::path pathObj(filePath);
        // Check if path exists and is of a directory file
        if (boost::filesystem::exists(pathObj))
            return true;
    }
    catch (boost::filesystem::filesystem_error & e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}


void FilterPCVoxelGrid(pcl::PointCloud<PointTFull>::Ptr& cloud, pcl::PointCloud<PointTFull>::Ptr& cloud_filtered, const float leaf_size)
{
    if (cloud == nullptr || cloud_filtered == nullptr)
    {
        ROS_ERROR("Null point cloud pointer");
        return;
    }

    if (!cloud->is_dense)
    {
        std::vector<int> map;
        pcl::removeNaNFromPointCloud(*(cloud),*(cloud),map);
        cloud->is_dense = true;
    }

    if (cloud->points.size() > 0)
    {   
        pcl::VoxelGrid<PointTFull> vg;
        vg.setLeafSize (leaf_size, leaf_size, leaf_size);
        vg.setInputCloud (cloud);
        vg.filter (*cloud_filtered);
    }
}


bool WriteMeshToPLY (const std::string& file_name, const pcl::PolygonMesh::Ptr& mesh)
{
    if (mesh == nullptr)
    {
        ROS_ERROR("Null input mesh pointer!");
        return false;
    }

    if (!pcl::io::savePolygonFilePLY (file_name, *mesh, false))
    {
        ROS_ERROR("Fail to save mesh!");
        return false;
    }
    else
        return true;
}


bool ReadMeshFromPLY (const std::string& file_name, pcl::PolygonMesh::Ptr& mesh)
{
    if (mesh == nullptr)
    {
        ROS_ERROR("Null input mesh pointer!");
        return false;
    }

    std::fstream fileStream(file_name);
    // fileStream.open(file_name);
    if (!fileStream.is_open()) 
    {
        ROS_ERROR("Fail to open file!");
        return false;
    }
    else
    {
        if (pcl::io::loadPolygonFilePLY (file_name, *mesh) == -1)
        {
            ROS_ERROR("Fail to load mesh!");
            return false;
        }
        else
            return true;

    }
}


bool ReadCloudFromPLY (const std::string& file_name, pcl::PointCloud<PointTFull>::Ptr& cloud, const bool if_filter=true, const float leaf_size=0.005)
{
    if (cloud == nullptr)
    {
        ROS_ERROR("Null input point cloud pointer!");
        return false;
    }

    std::fstream fileStream;
    fileStream.open(file_name);
    if (fileStream.fail()) 
    {
        ROS_ERROR("Fail to load point cloud!");
        return false;
    }
    else
    {
        if (pcl::io::loadPLYFile(file_name, *cloud) == -1)
        {
            ROS_ERROR("Fail to load point cloud!");
            return false;
        }
        else
        {
            if (if_filter)
                FilterPCVoxelGrid(cloud, cloud, leaf_size);
            return true;
        }  
    }
}


bool ReadCloudFromPLY (const std::string& file_name, pcl::PointCloud<PointTLabel>::Ptr& cloud)
{
    if (cloud == nullptr)
    {
        ROS_ERROR("Null input point cloud pointer!");
        return false;
    }

    std::fstream fileStream;
    fileStream.open(file_name);
    if (fileStream.fail()) 
    {
        ROS_ERROR("Fail to load point cloud!");
        return false;
    }
    else
    {
        if (pcl::io::loadPLYFile(file_name, *cloud) == -1)
        {
            ROS_ERROR("Fail to load point cloud!");
            return false;
        }
        else
            return true;
    }
}


bool WriteCloudToPLY (const std::string& file_name, const pcl::PointCloud<PointTFull>::Ptr& cloud)
{
    if (cloud == nullptr)
    {
        ROS_ERROR("Null input point cloud pointer!");
        return false;
    }

    if (!pcl::io::savePLYFile (file_name, *cloud, false))
    {
        ROS_ERROR("Fail to save cloud!");
        return false;
    }
    else
        return true;
}


bool ReadMeshFromOBJ (const std::string& file_name, pcl::PolygonMesh::Ptr& mesh)
{
    if (mesh == nullptr)
    {
        ROS_ERROR("Null input mesh pointer!");
        return false;
    }

    std::fstream fileStream;
    fileStream.open(file_name);
    if (fileStream.fail()) 
    {
        ROS_ERROR("Fail to open file!");
        return false;
    }
    else
    {
        if (pcl::io::loadOBJFile (file_name, *mesh) == -1)
        {
            ROS_ERROR("Fail to load mesh!");
            return false;
        }
        else
            return true;
    }
}


// void ToObjectNode(const Obj3D::Ptr object, pgm::ObjectNode::Ptr object_node)
// {
//     if (object == nullptr || object_node == nullptr)
//     {
//         ROS_ERROR("Null object pointer");
//         return;
//     }

//     pgm::Point pos (object->pos(0), object->pos(1), object->pos(2));
//     pgm::Quaternion quat (object->quat.x(), object->quat.y(), object->quat.z(), object->quat.w());
//     pgm::Point dims (object->aligned_dims(0), object->aligned_dims(1), object->aligned_dims(2));

//     object_node->setPose (pos, quat);
//     object_node->setBBox (dims);
// }


void ExtractCloudIndicesByLabel (const pcl::PointCloud<PointTLabel>::Ptr& cloud, pcl::PointIndices::Ptr& extracted_indices, const int label)
{
    if (cloud == nullptr || extracted_indices == nullptr)
    {
        ROS_ERROR("Null input cloud pointer!");
        return;
    }
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (cloud->points[i].label == label)
            extracted_indices->indices.push_back(i);
    }
}


void ExtractCloudByIndices (const pcl::PointCloud<PointTLabel>::Ptr& cloud, const pcl::PointIndices::Ptr& extracted_indices, pcl::PointCloud<PointTLabel>::Ptr& extracted_cloud)
{
    pcl::ExtractIndices<PointTLabel> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (extracted_indices);
    extract.setNegative (false);
    extract.filter (*extracted_cloud);
}


void ExtractCloudByIndices (const pcl::PointCloud<PointTFull>::Ptr& cloud, const pcl::PointIndices::Ptr& extracted_indices, pcl::PointCloud<PointTFull>::Ptr& extracted_cloud)
{
    pcl::ExtractIndices<PointTFull> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (extracted_indices);
    extract.setNegative (false);
    extract.filter (*extracted_cloud);
}


void ExtractCloudByLabel (const pcl::PointCloud<PointTLabel>::Ptr& cloud, pcl::PointCloud<PointTLabel>::Ptr& extracted_cloud, const int label)
{
    pcl::PointIndices::Ptr extracted_indices (new pcl::PointIndices);
    ExtractCloudIndicesByLabel (cloud, extracted_indices, label);
    ExtractCloudByIndices(cloud, extracted_indices, extracted_cloud);
}

}


#endif