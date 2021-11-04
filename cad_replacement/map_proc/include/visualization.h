#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "common.h" 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/keyboard_event.h>

namespace MapProcessing
{
class Viewer
{

public: 
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer ();
    ~Viewer(){};

    void KeyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* v);
    void VisualizeOnce (const std::vector<int>& mesh_id = {});
    void AddPointClouds (const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds);
    void AddPointClouds (const std::vector<pcl::PointCloud<PointT>::Ptr>& clouds);
    void AddPolygonMeshes (const std::vector<pcl::PolygonMesh::Ptr>& meshes, const std::vector<int>& mesh_id);
    void AddPlanes (const std::vector<Eigen::Vector4f>& planes);
    void AddCubes (const std::vector<OBBox>& boxes);


private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    bool viz_flag_ = true;


};

}


#endif