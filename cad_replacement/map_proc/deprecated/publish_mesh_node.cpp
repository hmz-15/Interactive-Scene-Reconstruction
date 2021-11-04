#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <mesh_msgs/TriangleMeshStamped.h>

#include "io.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "mesh_publisher");
    ros::start();

    ros::NodeHandle node_handle("~"); // resolve namespace of node handle
    
    std::string file;
    file = "/home/muzhi/231.ply";
    node_handle.getParam("input", file);
    ros::Publisher mesh_pub = node_handle.advertise<mesh_msgs::TriangleMeshStamped>("scene_mesh", 1);

    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
    bool sucess = MapProcessing::ReadMeshFromPLY (file, mesh);
    std::cout << sucess<< std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

    // while (1)
    // {
        mesh_msgs::TriangleMeshStamped mesh_stamped_msg;
        std::vector<mesh_msgs::TriangleIndices> triangles;
        std::vector<geometry_msgs::Point> vertices;
        std::vector<std_msgs::ColorRGBA> vertex_colors;
        std::vector<std_msgs::ColorRGBA> triangle_colors;

        for (int i = 0; i < mesh->polygons.size(); i++)
        {
            mesh_msgs::TriangleIndices indice;
            std_msgs::ColorRGBA color;
            indice.vertex_indices[0] = mesh->polygons[i].vertices[0];
            indice.vertex_indices[1] = mesh->polygons[i].vertices[1];
            indice.vertex_indices[2] = mesh->polygons[i].vertices[2];
            color.r = (float)cloud->points[mesh->polygons[i].vertices[0]].r/(float)255;
            color.g = (float)cloud->points[mesh->polygons[i].vertices[0]].g/(float)255;
            color.b = (float)cloud->points[mesh->polygons[i].vertices[0]].b/(float)255;
            color.a = (float)cloud->points[mesh->polygons[i].vertices[0]].a/(float)255;
            triangles.push_back(indice);
            triangle_colors.push_back(color);
        }
        std::cout << mesh->polygons.size() <<std::endl;
        mesh_stamped_msg.mesh.triangles = triangles;
        // mesh_stamped_msg.mesh.triangle_colors = triangle_colors;

        for (int i = 0; i < cloud->size(); i++)
        {
            // rviz use bgr by default
            geometry_msgs::Point point;
            std_msgs::ColorRGBA color;
            point.x = cloud->points[i].x;
            point.y = cloud->points[i].y;
            point.z = cloud->points[i].z;
            color.b = (float)cloud->points[i].b/(float)255;
            color.g = (float)cloud->points[i].g/(float)255;
            color.r = (float)cloud->points[i].r/(float)255;
            color.a = (float)cloud->points[i].a/(float)255;
            vertices.push_back(point);
            vertex_colors.push_back(color);
        }
        std::cout << cloud->size() <<std::endl;
        mesh_stamped_msg.mesh.vertices = vertices;
        mesh_stamped_msg.mesh.vertex_colors = vertex_colors;

        mesh_stamped_msg.header.frame_id = "map";
        mesh_stamped_msg.header.stamp = ros::Time::now();

        mesh_pub.publish(mesh_stamped_msg);


        std::cout << "sucess!" <<std::endl;

        ros::spin(); // spin() will not return until the node has been shutdown

    // }

    
    
	

    ros::shutdown();

    return 0;
}