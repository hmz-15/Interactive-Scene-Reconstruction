#include "visualization.h"

namespace MapProcessing
{

Viewer::Viewer ()
{
    viewer_.reset (new pcl::visualization::PCLVisualizer ("Map Processing Viewer"));
    viewer_->setBackgroundColor (1, 1, 1);
    viewer_->addCoordinateSystem (1.0);
    // viewer->initCameraParameters ();
    viewer_->registerKeyboardCallback(&Viewer::KeyboardEventOccurred, *this);
}


void Viewer::KeyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* v)
{
    if (event.getKeySym() == "a" && event.keyDown())
    {
        viz_flag_ = false;
    }
    // else if (event.getKeySym() == "b" && event.keyDown())
    // {
    //     viz_flag_ = false;
    //     terminate_flag_ = true;        
    // }     
}


void Viewer::VisualizeOnce (const std::vector<int>& mesh_id)
{
    while (viz_flag_)
    {
        ros::spinOnce ();
        ros::Duration (0.001).sleep ();
        viewer_->spinOnce (10); 
    }
    viz_flag_ = true;

    viewer_->removeAllShapes();
    viewer_->removeAllPointClouds();
    for (int i = 0; i < mesh_id.size(); i++)
        viewer_->removePolygonMesh("mesh "+ std::to_string(mesh_id[i]));
}


void Viewer::AddPointClouds (const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds)
{
    for (int i = 0; i < clouds.size(); i++)
        viewer_->addPointCloud(clouds[i], "pc "+ std::to_string(i));
}


void Viewer::AddPointClouds (const std::vector<pcl::PointCloud<PointT>::Ptr>& clouds)
{
    for (int i = 0; i < clouds.size(); i++)
        viewer_->addPointCloud(clouds[i], "pc "+ std::to_string(i));
}


void Viewer::AddPolygonMeshes (const std::vector<pcl::PolygonMesh::Ptr>& meshes, const std::vector<int>& mesh_id)
{
    assert(meshes.size() == mesh_id.size());
    for (int i = 0; i < meshes.size(); i++)
        viewer_->addPolygonMesh(*meshes[i], "mesh "+ std::to_string(mesh_id[i]));
}


void Viewer::AddPlanes (const std::vector<Eigen::Vector4f>& planes)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    for (int i = 0; i < planes.size(); i++)
    {
        coefficients->values = {planes[i](0), planes[i](1), planes[i](2), planes[i](3)}; 
        viewer_->addPlane(*coefficients, "plane " + std::to_string(i));
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "plane "+std::to_string(i));
    } 
}


void Viewer::AddCubes (const std::vector<OBBox>& boxes)
{   
    for (int i = 0; i < boxes.size(); i++)
    {
        viewer_->addCube (boxes[i].pos, boxes[i].quat, boxes[i].aligned_dims(0), boxes[i].aligned_dims(1), boxes[i].aligned_dims(2), "OBB " + std::to_string(i));
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB " + std::to_string(i));
    }
}

}
