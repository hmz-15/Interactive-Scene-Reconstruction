#include "collision_check.h"

namespace MapProcessing
{

CollisionObjectf* PCLMeshToFCLObject(const pcl::PolygonMesh::Ptr& mesh)
{
    if (mesh == nullptr)
    {
        ROS_ERROR("Null input mesh pointer!");
        return nullptr;
    }
    // Get vertices 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

    // set mesh triangles and vertice indices
    std::vector<Vector3f> vertices;
    std::vector<Triangle> triangles;
    for (int i = 0; i < mesh->polygons.size(); i++)
    {
        Triangle tri;
        tri[0] = mesh->polygons[i].vertices[0];
        tri[1] = mesh->polygons[i].vertices[1];
        tri[2] = mesh->polygons[i].vertices[2];
        triangles.push_back(tri);
    }
    for (int i = 0; i < cloud->size(); i++)
    {
        Vector3f vert;
        vert[0] = cloud->points[i].x;
        vert[1] = cloud->points[i].y;
        vert[2] = cloud->points[i].z;
        vertices.push_back(vert);
    }
    // Create collision object
    std::shared_ptr<BVHModel<OBBRSSf>> geom = std::make_shared<BVHModel<OBBRSSf>>();
    geom->beginModel();
    geom->addSubModel(vertices, triangles);
    geom->endModel();
    CollisionObjectf* obj = new CollisionObjectf(geom);
    return obj;
}


CollisionObjectf* PlaneToFCLObject(const Eigen::Vector4f& plane)
{
    std::shared_ptr<Planef> geom = std::make_shared<Planef>(plane[0], plane[1], plane[2], -plane[3]);
    CollisionObjectf* obj = new CollisionObjectf(geom);
    return obj;
}


CollisionObjectf* HalfSpaceToFCLObject(const Eigen::Vector4f& plane)
{
    std::shared_ptr<Halfspacef> geom = std::make_shared<Halfspacef>(-plane[0], -plane[1], -plane[2], plane[3]);
    CollisionObjectf* obj = new CollisionObjectf(geom);
    return obj;
}


bool ComputeMeshPlaneCollision(const pcl::PolygonMesh::Ptr& mesh, const Eigen::Vector4f& plane, std::vector<Eigen::Vector3f>& contact_locations, bool return_details)
{
    if (mesh == nullptr)
    {
        ROS_ERROR("Null input pointer!");
        return false;
    }

    // Mesh collision object in FCL
    CollisionObjectf* mesh_obj = PCLMeshToFCLObject(mesh);
    CollisionObjectf* collide_plane = PlaneToFCLObject(plane);

    // Set the collision request structure
    CollisionRequestf request;
    request.enable_contact = return_details;
    request.num_max_contacts = 200;
    // Result will be returned via the collision result structure
    CollisionResultf result;
    // Perform collision test
    collide(mesh_obj, collide_plane, request, result);
    // Get the contact info if requested
    if (return_details)
    {
        std::vector<Contactf> contacts;
        result.getContacts(contacts);
        for (auto& contact: contacts)
        {
            Vector3f pos = contact.pos;
            contact_locations.push_back(Eigen::Vector3f(pos[0], pos[1], pos[2]));
        }
    }
    delete mesh_obj, collide_plane;
    return result.isCollision();
}


bool ComputeMeshHalfspaceCollision(const pcl::PolygonMesh::Ptr& mesh, const Eigen::Vector4f& plane, std::vector<Eigen::Vector3f>& contact_locations, bool return_details)
{
    if (mesh == nullptr)
    {
        ROS_ERROR("Null input pointer!");
        return false;
    }

    // Mesh collision object in FCL
    CollisionObjectf* mesh_obj = PCLMeshToFCLObject(mesh);
    CollisionObjectf* collide_halfspace = HalfSpaceToFCLObject(plane);

    // Set the collision request structure
    CollisionRequestf request;
    request.enable_contact = return_details;
    request.num_max_contacts = 200;
    // Result will be returned via the collision result structure
    CollisionResultf result;
    // Perform collision test
    collide(mesh_obj, collide_halfspace, request, result);
    // Get the contact info if requested
    if (return_details)
    {
        std::vector<Contactf> contacts;
        result.getContacts(contacts);
        for (auto& contact: contacts)
        {
            Vector3f pos = contact.pos;
            contact_locations.push_back(Eigen::Vector3f(pos[0], pos[1], pos[2]));
        }
    }
    delete mesh_obj, collide_halfspace;
    return result.isCollision();
}


bool ComputeMeshMeshCollision(const pcl::PolygonMesh::Ptr& mesh_1, const pcl::PolygonMesh::Ptr& mesh_2, std::vector<Eigen::Vector3f>& contact_locations, std::vector<float>& penetration_depths,
                                bool return_details)
{
    if (mesh_1 == nullptr || mesh_2 == nullptr)
    {
        ROS_ERROR("Null input pointer!");
        return false;
    }

    // Mesh collision object in FCL
    CollisionObjectf* mesh_obj_1 = PCLMeshToFCLObject(mesh_1);
    CollisionObjectf* mesh_obj_2 = PCLMeshToFCLObject(mesh_2);

    // Set the collision request structure
    CollisionRequestf request;
    request.enable_contact = return_details;
    request.num_max_contacts = 200;
    // Result will be returned via the collision result structure
    CollisionResultf result;
    // Perform collision test
    collide(mesh_obj_1, mesh_obj_2, request, result);
    // Get the contact info if requested
    if (return_details)
    {
        std::vector<Contactf> contacts;
        result.getContacts(contacts);
        for (auto& contact: contacts)
        {
            Vector3f pos = contact.pos;
            float penetration_depth = contact.penetration_depth;
            contact_locations.push_back(Eigen::Vector3f(pos[0], pos[1], pos[2]));
            penetration_depths.push_back(penetration_depth);
        }
    }
    delete mesh_obj_1, mesh_obj_2;

    return result.isCollision();
}


float ComputeMeshMeshDistance(const pcl::PolygonMesh::Ptr& mesh_1, const pcl::PolygonMesh::Ptr& mesh_2)
{
    if (mesh_1 == nullptr || mesh_2 == nullptr)
    {
        ROS_ERROR("Null input pointer!");
        return false;
    }

    // Mesh collision object in FCL
    CollisionObjectf* mesh_obj_1 = PCLMeshToFCLObject(mesh_1);
    CollisionObjectf* mesh_obj_2 = PCLMeshToFCLObject(mesh_2);

    // set the distance request structure, here we just use the default setting
    DistanceRequestf request;
    // result will be returned via the collision result structure
    DistanceResultf result;
    // perform distance test
    distance(mesh_obj_1, mesh_obj_2, request, result);
    delete mesh_obj_1, mesh_obj_2;

    return result.min_distance;
}


float ComputeCloudCloudDistance (const pcl::PointCloud<PointTFull>::Ptr cloud_1, const pcl::PointCloud<PointTFull>::Ptr cloud_2)
{
    if (cloud_1 == nullptr || cloud_2 == nullptr)
    {
        ROS_ERROR("Null point cloud pointer");
        return 0;
    } 

    float squared_dist = 10.0f;
    int k = 1;
    pcl::KdTreeFLANN<PointTFull> kdtree;
    kdtree.setInputCloud (cloud_2);

    for (int i = 0; i < cloud_1->points.size(); i++)
    {
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        if (kdtree.nearestKSearch(cloud_1->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            if (pointNKNSquaredDistance[0] < squared_dist)
                squared_dist = pointNKNSquaredDistance[0];            
    }

    return sqrt(squared_dist);
}


}