#include <iostream>
#include <ros/ros.h>
#include "common.h"
#include "io.h"
#include "visualization.h"
#include "collision_check.h"

namespace MapProcessing
{

void test()
{
    Eigen::Vector4f plane(0, 0, 1, 0);

    std::cout << 0 << std::endl;

    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
    ReadMeshFromOBJ ("/home/muzhi/Dataset/ShapeNetSem/models-OBJ/models-aligned/111720e8cd4c613492d9da2668ec34c.obj", mesh);

    std::vector<Eigen::Vector3f> contact_locations;
    bool if_contact = ComputeMeshPlaneCollision(mesh, plane, contact_locations, true); 
    std::cout << contact_locations.size() << std::endl;
    for (auto loc: contact_locations)
        std::cout << loc << std::endl;

    Viewer::Ptr viewer = std::make_shared<Viewer> (); 
    viewer->AddPolygonMeshes ({mesh}, {0});
    viewer->AddPlanes ({plane});
    viewer->VisualizeOnce({0});
}

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "collision_test_node");
    ros::start();
    std::cout << "Start testing!" << std::endl;
    MapProcessing::test();

    ros::shutdown();
    return 0;
    
}