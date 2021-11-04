#ifndef COLLISION_CHECK_H
#define COLLISION_CHECK_H

#include "common.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/distance.h"

using namespace fcl;

namespace MapProcessing
{

CollisionObjectf* PCLMeshToFCLObject(const pcl::PolygonMesh::Ptr& mesh);

CollisionObjectf* PlaneToFCLObject(const Eigen::Vector4f& plane);

CollisionObjectf* HalfSpaceToFCLObject(const Eigen::Vector4f& plane);

bool ComputeMeshPlaneCollision(const pcl::PolygonMesh::Ptr& mesh, const Eigen::Vector4f& plane, std::vector<Eigen::Vector3f>& contact_locations, bool return_details = false);

bool ComputeMeshHalfspaceCollision(const pcl::PolygonMesh::Ptr& mesh, const Eigen::Vector4f& plane, std::vector<Eigen::Vector3f>& contact_locations, bool return_details = false);

bool ComputeMeshMeshCollision(const pcl::PolygonMesh::Ptr& mesh_1, const pcl::PolygonMesh::Ptr& mesh_2, std::vector<Eigen::Vector3f>& contact_locations, std::vector<float>& penetration_depths,
                                bool return_details = false);

float ComputeMeshMeshDistance(const pcl::PolygonMesh::Ptr& mesh_1, const pcl::PolygonMesh::Ptr& mesh_2);

float ComputeCloudCloudDistance (const pcl::PointCloud<PointTFull>::Ptr cloud_1, const pcl::PointCloud<PointTFull>::Ptr cloud_2);


}





#endif