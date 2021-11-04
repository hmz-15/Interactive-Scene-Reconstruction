#ifndef UTILS_H_
#define UTILS_H_

#include "common.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// OpenMP
#include <omp.h>

#include <ApproxMVBB/ComputeApproxMVBB.hpp>

namespace MapProcessing
{

void ComputeNormal(pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud, const float radius);

Eigen::Vector3f ComputeMeanNormal(const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud);


float ComputeNormalError (const PointTFull& point_1, const PointTFull& point_2);


void ComputePlanesRANSAC(const pcl::PointCloud<PointTFull>::Ptr& cloud, std::vector<Eigen::Vector4f>& planes, float distance_threshood);


bool CheckOverlap2D (const OBBox& parent, const OBBox& child, const int ground);

bool CheckOverlap2DRough (const OBBox& parent, const OBBox& child, const int ground);


float GetOverlapRatio2D (const OBBox& parent, const OBBox& child, const int ground, float step_size=0.005);

float GetOverlap1D (Eigen::Vector2f& range_1, Eigen::Vector2f& range_2);

float ComputeGroundedBoxIOU3D (const OBBox& box_1, const OBBox& box_2, const int ground);


bool IsSupportingPlane (const Eigen::Vector4f& plane, const Eigen::Vector3f& ground_axis, const float threshood=0.9);

float ComputePlaneError (const Eigen::Vector4f& plane1, const Eigen::Vector4f& plane2, const float scale=1.0);

void BatchTransformPlanesGlobalToLocal (const std::vector<Eigen::Vector4f>& planes, const std::vector<Eigen::Matrix4f>& transforms,
                                                        std::vector<std::vector<Eigen::Vector4f>>& transformed_planes);

Eigen::Vector4f TransformPlaneGlobalToLocal (const Eigen::Vector4f& plane, const Eigen::Matrix4f& Tg_l);


Eigen::Vector4f TransformPlaneLocalToGlobal (const Eigen::Vector4f& plane, const Eigen::Matrix4f& Tg_l);


void GenerateCanonicalBaseTransforms (std::vector<Eigen::Matrix4f>& canonical_base_transforms);


void GetCanonicalTransforms (const std::vector<Eigen::Matrix4f>& canonical_base_transforms, const Eigen::Matrix4f& init_transform,
                                std::vector<Eigen::Matrix4f>& canonical_transforms);


Eigen::Matrix4f GetHomogeneousTransformMatrix (const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, const float scale=1.0);


void TransformMesh (const pcl::PolygonMesh::Ptr& input_mesh, Eigen::Matrix4f& transform_matrix,  pcl::PolygonMesh::Ptr& output_mesh);

void TransformMeshFull (const pcl::PolygonMesh::Ptr& input_mesh, Eigen::Matrix4f& transform_matrix,  pcl::PolygonMesh::Ptr& output_mesh);

int CheckPointCloudCollision (const pcl::PointCloud<PointT>::Ptr cloud_1, const pcl::PointCloud<PointT>::Ptr cloud_2);

float ComputePointCloudToMeshError(const pcl::PointCloud<PointTFull>& point_cloud, const pcl::PolygonMesh& mesh);

float ComputePointCloudOverlap (const pcl::PointCloud<PointTFull>::Ptr cloud_1, const pcl::PointCloud<PointTFull>::Ptr cloud_2, const float dist_threshood);

float ComputePointCloudOverlapSquaredDistance (const pcl::PointCloud<PointTFull>::Ptr cloud_1, const pcl::PointCloud<PointTFull>::Ptr cloud_2, const float dist_threshood);

void ComputeGroundOrientedBoundingBox(const pcl::PointCloud<PointTFull>::Ptr& cloud, OBBox& box, const int ground);

void ComputeGroundOrientedBoundingBox(const pcl::PointCloud<PointT>::Ptr& cloud, OBBox& box, const int ground);

float ComputeBoxPlanePenetration (const OBBox& box, const Eigen::Vector4f& plane);

}

#endif