#include "utils.h"
#include "3rd_party/triangle_point/poitri.h"


namespace MapProcessing
{

void ComputeNormal(pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud, const float radius)
{
    if (cloud == nullptr || normal_cloud == nullptr)
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

    pcl::NormalEstimationOMP <PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree_normal (new pcl::search::KdTree<PointT>);

    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree_normal);
    ne.setRadiusSearch (radius);

    // Compute the features
    ne.compute (*normal_cloud);
    // pcl::concatenateFields (*cloud, *cloud_normals, *normal_cloud);
}


Eigen::Vector3f ComputeMeanNormal(const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud)
{
    if (normal_cloud == nullptr)
    {
        ROS_ERROR("Null point cloud pointer");
        return Eigen::Vector3f(0.0, 0.0, 0.0);
    }

    Eigen::Vector3f normal (0.0, 0.0, 0.0);
    for (int i = 0; i < normal_cloud->points.size(); i++)
    {
        Eigen::Vector3f point_normal (normal_cloud->points[i].normal_x, normal_cloud->points[i].normal_y, normal_cloud->points[i].normal_z);
        normal += point_normal;
    }
    normal /= normal_cloud->points.size();
    return normal;
}


float ComputeNormalError (const PointTFull& point_1, const PointTFull& point_2)
{
    Eigen::Vector3f normals_1 (point_1.normal_x, point_1.normal_y, point_1.normal_z);
    Eigen::Vector3f normals_2 (point_2.normal_x, point_2.normal_y, point_2.normal_z);

    return (1.0 - normals_1.transpose()*normals_2);
}


void ComputePlanesRANSAC(const pcl::PointCloud<PointTFull>::Ptr& cloud, std::vector<Eigen::Vector4f>& planes, float distance_threshood)
{
    if (cloud == nullptr)
    {
        ROS_ERROR("Null input point cloud pointer!");
        return;
    }
        
    planes.clear();
    int i = 1;

    pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT>);   
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);  
    pcl::copyPointCloud(*cloud, *input_cloud);
    pcl::copyPointCloud(*cloud, *normal_cloud);

    int input_cloud_size = input_cloud->points.size();
    std::cout << "num of points: " << input_cloud_size <<std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    // seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshood);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (200);

    while (1)
    {   
        seg.setInputCloud (input_cloud);
        seg.setInputNormals (normal_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () > 0.1 * input_cloud_size)
        {
            // Extract outliers
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud (input_cloud);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*input_cloud);

            pcl::ExtractIndices<pcl::Normal> extract_normal;
            extract_normal.setInputCloud (normal_cloud);
            extract_normal.setIndices (inliers);
            pcl::PointCloud<pcl::Normal>::Ptr normal_inliers (new pcl::PointCloud<pcl::Normal>);  
            extract_normal.setNegative (false);
            extract_normal.filter (*normal_inliers);

            extract_normal.setIndices (inliers);
            extract_normal.setNegative (true);
            // extract_normal.setKeepOrganized (true);
            extract_normal.filter (*normal_cloud);

            // Check direction of plane normal
            Eigen::Vector4f plane (coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
            Eigen::Vector3f normal = ComputeMeanNormal (normal_inliers);
            if (normal.transpose() * plane.head(3) < 0)
                plane = -plane;
            planes.push_back(plane);
            
            // std::cout << "Iteration: " + std::to_string(i) <<std::endl;
            // std::cout << "Model coefficients: " << plane(0) << " " 
            //                                     << plane(1) << " "
            //                                     << plane(2) << " " 
            //                                     << plane(3) << std::endl;
            // std::cout << "Model inliers: " << inliers->indices.size () << std::endl;

            if (input_cloud->points.size() < 0.1 * input_cloud_size)
                break;
            i++;
        }
        else
            break;
    }
}

bool CheckOverlap2DRough (const OBBox& parent, const OBBox& child, const int ground)
{
    // For objects on the ground
    Eigen::Vector2f parent_center (parent.pos((ground+1)%3), parent.pos((ground+2)%3));
    Eigen::Vector2f parent_half_dims (parent.aligned_dims((ground+1)%3)/2, parent.aligned_dims((ground+2)%3)/2);
    Eigen::Vector2f child_center (child.pos((ground+1)%3), child.pos((ground+2)%3));
    Eigen::Vector2f child_half_dims (child.aligned_dims((ground+1)%3)/2, child.aligned_dims((ground+2)%3)/2);

    Eigen::Vector2f vect = parent_center - child_center;
    if (vect.norm() > parent_half_dims.norm() + child_half_dims.norm())
        return false;
    else
        return true;
}

bool CheckOverlap2D (const OBBox& parent, const OBBox& child, const int ground)
{
    // For parent object
    Eigen::Matrix3f parent_rot = parent.quat.toRotationMatrix();
    Eigen::Vector2f parent_center (parent.pos((ground+1)%3), parent.pos((ground+2)%3));
    Eigen::Vector2f parent_u (parent_rot((ground+1)%3, (ground+1)%3), parent_rot((ground+2)%3, (ground+1)%3));
    Eigen::Vector2f parent_v (parent_rot((ground+1)%3, (ground+2)%3), parent_rot((ground+2)%3, (ground+2)%3));

    // For child object
    Eigen::Matrix3f child_rot = child.quat.toRotationMatrix();
    Eigen::Vector2f child_center (child.pos((ground+1)%3), child.pos((ground+2)%3));
    Eigen::Vector2f child_u (child_rot((ground+1)%3, (ground+1)%3), child_rot((ground+2)%3, (ground+1)%3));
    Eigen::Vector2f child_v (child_rot((ground+1)%3, (ground+2)%3), child_rot((ground+2)%3, (ground+2)%3));

    float step_size = 0.01;
    int max_step_child_u = (int)(child.aligned_dims((ground+1)%3)/2/step_size);
    int max_step_child_v = (int)(child.aligned_dims((ground+2)%3)/2/step_size);
    float max_dim_parent_u = parent.aligned_dims((ground+1)%3)/2;
    float max_dim_parent_v = parent.aligned_dims((ground+2)%3)/2;

    for (int i = -max_step_child_u; i < max_step_child_u; i++)
    {
        for (int j = -max_step_child_v; j < max_step_child_v; j++)
        {
            Eigen::Vector2f grid = child_center + i * child_u * step_size + j * child_v * step_size;
            Eigen::Vector2f vect = grid - parent_center;
            float grid_u = vect.transpose()*parent_u;
            float grid_v = vect.transpose()*parent_v;
            if (grid_u < max_dim_parent_u && grid_u > -max_dim_parent_u && grid_v < max_dim_parent_v && grid_v > -max_dim_parent_v)
               return true;
        }
    }
    return false;
}


float GetOverlapRatio2D (const OBBox& parent, const OBBox& child, const int ground, float step_size)
{
    // For objects on the ground
    // For parent object
    Eigen::Matrix3f parent_rot = parent.quat.toRotationMatrix();
    Eigen::Vector2f parent_center (parent.pos((ground+1)%3), parent.pos((ground+2)%3));
    Eigen::Vector2f parent_u (parent_rot((ground+1)%3, (ground+1)%3), parent_rot((ground+2)%3, (ground+1)%3));
    Eigen::Vector2f parent_v (parent_rot((ground+1)%3, (ground+2)%3), parent_rot((ground+2)%3, (ground+2)%3));

    // For child object
    Eigen::Matrix3f child_rot = child.quat.toRotationMatrix();
    Eigen::Vector2f child_center (child.pos((ground+1)%3), child.pos((ground+2)%3));
    Eigen::Vector2f child_u (child_rot((ground+1)%3, (ground+1)%3), child_rot((ground+2)%3, (ground+1)%3));
    Eigen::Vector2f child_v (child_rot((ground+1)%3, (ground+2)%3), child_rot((ground+2)%3, (ground+2)%3));

    int max_step_child_u = (int)(child.aligned_dims((ground+1)%3)/2/step_size);
    int max_step_child_v = (int)(child.aligned_dims((ground+2)%3)/2/step_size);
    while (std::min(max_step_child_u, max_step_child_v) <= 4)
    {
        step_size /= 2.0f;
        max_step_child_u = (int)(child.aligned_dims((ground+1)%3)/2/step_size);
        max_step_child_v = (int)(child.aligned_dims((ground+2)%3)/2/step_size);
    }

    float max_dim_parent_u = parent.aligned_dims((ground+1)%3)/2;
    float max_dim_parent_v = parent.aligned_dims((ground+2)%3)/2;

    int overlap_count = 0;

    for (int i = -max_step_child_u; i < max_step_child_u; i++)
    {
        for (int j = -max_step_child_v; j < max_step_child_v; j++)
        {
            Eigen::Vector2f grid = child_center + i * child_u * step_size + j * child_v * step_size;
            Eigen::Vector2f vect = grid - parent_center;
            float grid_u = vect.transpose()*parent_u;
            float grid_v = vect.transpose()*parent_v;
            if (grid_u < max_dim_parent_u && grid_u > -max_dim_parent_u && grid_v < max_dim_parent_v && grid_v > -max_dim_parent_v)
               overlap_count++; 
        }
    }
    float overlap_ratio = (float)(overlap_count)/(float)(max_step_child_u*max_step_child_v*4);

    return overlap_ratio;
}


float GetOverlap1D (Eigen::Vector2f& range_1, Eigen::Vector2f& range_2)
{
    if (range_1(0) > range_1(1) || range_2(0) > range_2(1))
    {
        ROS_ERROR("Input range should be from smaller value to larger one!");
        return 0.0;
    }

    Eigen::Vector2f larger_range, smaller_range;
    if (range_1.mean() > range_2.mean())
    {
        larger_range = range_1;
        smaller_range = range_2;
    }
    else
    {
        larger_range = range_2;
        smaller_range = range_1;
    }

    float lower_bound = std::max(larger_range(0), smaller_range(0));
    float higher_bound = std::min(larger_range(1), smaller_range(1));

    if (higher_bound > lower_bound)
        return (higher_bound - lower_bound);
    else
        return 0;
}


float ComputeGroundedBoxIOU3D (const OBBox& box_1, const OBBox& box_2, const int ground)
{
    float overlap_ratio_1 = GetOverlapRatio2D (box_2, box_1, ground);
    float overlap_ratio_2 = GetOverlapRatio2D (box_1, box_2, ground);

    if (overlap_ratio_1 == 0.0 || overlap_ratio_2 == 0.0)
        return 0.0;

    Eigen::Vector2f height_range_1 (box_1.pos(ground) - box_1.aligned_dims(ground)/2, box_1.pos(ground) + box_1.aligned_dims(ground)/2);
    Eigen::Vector2f height_range_2 (box_2.pos(ground) - box_2.aligned_dims(ground)/2, box_2.pos(ground) + box_2.aligned_dims(ground)/2);
    float height_overlap = GetOverlap1D (height_range_1, height_range_2);

    float iou = 1.0 * height_overlap / (box_1.aligned_dims(ground) / overlap_ratio_1 + box_2.aligned_dims(ground) / overlap_ratio_2 - height_overlap);
    return iou;
}


// bool IsSamePlane (const Eigen::Vector4f& plane1, const Eigen::Vector4f& plane2)
// {
//     if (plane1.head(3).transpose() * plane2.head(3) > 0.95 && std::abs(plane1(3) - plane2(3) < 0.02))
//         return true;
//     else
//         return false;
// }


bool IsSupportingPlane (const Eigen::Vector4f& plane, const Eigen::Vector3f& ground_axis, const float threshood)
{
    if (plane.head(3).transpose() * ground_axis > threshood)
        return true;
    else
        return false;
}


float ComputePlaneError (const Eigen::Vector4f& plane1, const Eigen::Vector4f& plane2, const float scale)
{
    float error = (1.0 - plane1.head(3).transpose() * plane2.head(3)) + std::abs(scale * plane1(3) - plane2(3));
    return error;
}


void BatchTransformPlanesGlobalToLocal (const std::vector<Eigen::Vector4f>& planes, const std::vector<Eigen::Matrix4f>& transforms,
                                                        std::vector<std::vector<Eigen::Vector4f>>& transformed_planes)
{
    transformed_planes.clear();
    for (int i = 0; i < transforms.size(); i++)
    {
        std::vector<Eigen::Vector4f> single_transform_planes;
        for (int j = 0; j < planes.size(); j++)
        {
            Eigen::Vector4f current_plane = TransformPlaneGlobalToLocal(planes[j], transforms[i]);
            single_transform_planes.push_back(current_plane);
        }
        transformed_planes.push_back(single_transform_planes);
    }
}


Eigen::Vector4f TransformPlaneGlobalToLocal (const Eigen::Vector4f& plane, const Eigen::Matrix4f& Tg_l)
{
    Eigen::Vector4f transformed_plane = Tg_l.transpose()*plane;
    float len_normal = sqrt(transformed_plane.head(3).transpose()*transformed_plane.head(3));
    transformed_plane = transformed_plane / len_normal;
    return transformed_plane;
}


Eigen::Vector4f TransformPlaneLocalToGlobal (const Eigen::Vector4f& plane, const Eigen::Matrix4f& Tg_l)
{
    Eigen::Vector4f transformed_plane = Tg_l.transpose().inverse()*plane;
    float len_normal = sqrt(transformed_plane.head(3).transpose()*transformed_plane.head(3));
    transformed_plane = transformed_plane / len_normal;
    return transformed_plane;
}


void GenerateCanonicalBaseTransforms (std::vector<Eigen::Matrix4f>& canonical_base_transforms)
{
    canonical_base_transforms.clear();
    for (int i = 0; i < 3; i++)
    {
        for (int j = 1; j < 3; j++)
        {
            Eigen::Matrix4f transform_1;
            transform_1.setZero();
            transform_1(3,3) = 1.0;
            transform_1(i, 0) = 1.0;
            transform_1((i+j)%3, 1) = 1.0;
            transform_1.col(2).head(3) = Eigen::Vector3f(transform_1.col(0).head(3)).cross(Eigen::Vector3f(transform_1.col(1).head(3)));
            canonical_base_transforms.push_back(transform_1);

            Eigen::Matrix4f transform_2;
            transform_2.setZero();
            transform_2(3,3) = 1.0;
            transform_2(i, 0) = 1.0;
            transform_2((i+j)%3, 1) = -1.0;
            transform_2.col(2).head(3) = Eigen::Vector3f(transform_2.col(0).head(3)).cross(Eigen::Vector3f(transform_2.col(1).head(3)));
            canonical_base_transforms.push_back(transform_2);

            Eigen::Matrix4f transform_3;
            transform_3.setZero();
            transform_3(3,3) = 1.0;
            transform_3(i, 0) = -1.0;
            transform_3((i+j)%3, 1) = 1.0;
            transform_3.col(2).head(3) = Eigen::Vector3f(transform_3.col(0).head(3)).cross(Eigen::Vector3f(transform_3.col(1).head(3)));
            canonical_base_transforms.push_back(transform_3);

            Eigen::Matrix4f transform_4;
            transform_4.setZero();
            transform_4(3,3) = 1.0;
            transform_4(i, 0) = -1.0;
            transform_4((i+j)%3, 1) = -1.0;
            transform_4.col(2).head(3) = Eigen::Vector3f(transform_4.col(0).head(3)).cross(Eigen::Vector3f(transform_4.col(1).head(3)));
            canonical_base_transforms.push_back(transform_4);
        }
    }
}


void GetCanonicalTransforms (const std::vector<Eigen::Matrix4f>& canonical_base_transforms, const Eigen::Matrix4f& init_transform,
                                std::vector<Eigen::Matrix4f>& canonical_transforms)
{
    canonical_transforms.clear();

    for (int i = 0; i < canonical_base_transforms.size(); i++)
    {
        Eigen::Matrix4f current_transform = init_transform * canonical_base_transforms[i];
        canonical_transforms.push_back(current_transform);
    }
}


Eigen::Matrix4f GetHomogeneousTransformMatrix (const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, const float scale)
{
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.topLeftCorner(3, 3) = scale * quat.toRotationMatrix();
    transform_matrix.col(3).head(3) = pos;
    return transform_matrix;
}


void TransformMesh (const pcl::PolygonMesh::Ptr& input_mesh, Eigen::Matrix4f& transform_matrix,  pcl::PolygonMesh::Ptr& output_mesh)
{
    if (input_mesh == nullptr || output_mesh == nullptr)
    {
        ROS_ERROR("Null mesh pointer");
        return;
    }

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    // pcl::PointCloud<PointT>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(input_mesh->cloud, *cloud);
    // pcl::fromPCLPointCloud2(input_mesh->cloud, *color_cloud);
    pcl::transformPointCloud(*cloud, *cloud, transform_matrix);

    *output_mesh = *input_mesh;

    pcl::toPCLPointCloud2(*cloud, output_mesh->cloud);
}


void TransformMeshFull (const pcl::PolygonMesh::Ptr& input_mesh, Eigen::Matrix4f& transform_matrix,  pcl::PolygonMesh::Ptr& output_mesh)
{
    if (input_mesh == nullptr || output_mesh == nullptr)
    {
        ROS_ERROR("Null mesh pointer");
        return;
    }

    pcl::PointCloud<PointTFull>::Ptr cloud (new pcl::PointCloud<PointTFull>);
    // pcl::PointCloud<PointT>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(input_mesh->cloud, *cloud);
    // pcl::fromPCLPointCloud2(input_mesh->cloud, *color_cloud);
    pcl::transformPointCloud(*cloud, *cloud, transform_matrix);

    *output_mesh = *input_mesh;

    pcl::toPCLPointCloud2(*cloud, output_mesh->cloud);
}


int CheckPointCloudCollision (const pcl::PointCloud<PointT>::Ptr cloud_1, const pcl::PointCloud<PointT>::Ptr cloud_2)
{
    if (cloud_1 == nullptr || cloud_2 == nullptr)
    {
        ROS_ERROR("Null point cloud pointer");
        return 0;
    } 

    int collide_count = 0;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (cloud_2);
    for (int i = 0; i < cloud_1->points.size(); i++)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (kdtree.radiusSearch (cloud_1->points[i], 0.005, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            if (pointIdxRadiusSearch.size() > 0)
                collide_count++;
        }              
    }
    return collide_count;
}


float ComputePointCloudToMeshError(const pcl::PointCloud<PointTFull>& point_cloud, const pcl::PolygonMesh& mesh) 
{
    float error = 0;
    // Get vertices 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    #pragma omp parallel
    {
        #pragma omp for
        for (unsigned int i = 0; i < point_cloud.points.size(); i++)
        {
            float min_distance = FLT_MAX;
            Vec3f point (point_cloud.points[i].x, point_cloud.points[i].y, point_cloud.points[i].z);

            for (int j = 0; j < mesh.polygons.size(); j++) {
                Vec3f closest_point(0);

                if (mesh.polygons[j].vertices.size() < 3)
                    continue;

                int ind1 = mesh.polygons[j].vertices[0];
                int ind2 = mesh.polygons[j].vertices[1];
                int ind3 = mesh.polygons[j].vertices[2];

                Vec3f v1 (cloud->points[ind1].x, cloud->points[ind1].y, cloud->points[ind1].z);
                Vec3f v2 (cloud->points[ind2].x, cloud->points[ind2].y, cloud->points[ind2].z);
                Vec3f v3 (cloud->points[ind3].x, cloud->points[ind3].y, cloud->points[ind3].z);
                float distance = point_triangle_distance(point, v1, v2, v3, closest_point);
                // float distance = (point - closest_point).norm();

                if (distance < min_distance)
                    min_distance = distance;
            }
            #pragma omp atomic
            error += min_distance*min_distance;
      }
    }

    error /= point_cloud.points.size();
    error = sqrt(error);
    return error;
}


float ComputePointCloudOverlap (const pcl::PointCloud<PointTFull>::Ptr cloud_1, const pcl::PointCloud<PointTFull>::Ptr cloud_2, const float dist_threshood)
{
    if (cloud_1 == nullptr || cloud_2 == nullptr)
    {
        ROS_ERROR("Null point cloud pointer");
        return 0;
    } 

    int num_overlop_points = 0;
    pcl::KdTreeFLANN<PointTFull> kdtree;
    kdtree.setInputCloud (cloud_2);

    // auto cmp = [](const auto & a, const auto & b) -> bool { return a.second < b.second;}; 

    for (int i = 0; i < cloud_1->points.size(); i++)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (kdtree.radiusSearch (cloud_1->points[i], dist_threshood, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            // std::vector<std::pair<int, float>> index_distance_pairs;
            // for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
            //     index_distance_pairs.push_back(std::make_pair(pointIdxRadiusSearch[j], pointRadiusSquaredDistance[j]));

            // std::sort(index_distance_pairs.begin(), index_distance_pairs.end(), cmp);
            for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
            {
                if (ComputeNormalError (cloud_2->points[pointIdxRadiusSearch[j]], cloud_1->points[i]) < 0.5)
                {
                    num_overlop_points++;
                    break;
                }
            }
            // if (ComputeNormalError (cloud_2->points[index_distance_pairs[0].first], cloud_1->points[i] < ))

        }
            
    }

    return (float)num_overlop_points  / (float)cloud_1->points.size();
}


float ComputePointCloudOverlapSquaredDistance (const pcl::PointCloud<PointTFull>::Ptr cloud_1, const pcl::PointCloud<PointTFull>::Ptr cloud_2, const float dist_threshood)
{
    if (cloud_1 == nullptr || cloud_2 == nullptr)
    {
        ROS_ERROR("Null point cloud pointer");
        return 0;
    } 

    float squared_distance = 0.0f;
    pcl::KdTreeFLANN<PointTFull> kdtree;
    kdtree.setInputCloud (cloud_2);

    // auto cmp = [](const auto & a, const auto & b) -> bool { return a.second < b.second;}; 

    for (int i = 0; i < cloud_1->points.size(); i++)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (kdtree.radiusSearch (cloud_1->points[i], dist_threshood, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            std::sort(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end());
            squared_distance += pointRadiusSquaredDistance[0];
        }
        else
        {
            squared_distance += dist_threshood*dist_threshood;
        }    
    }

    return squared_distance/ (float) cloud_1->points.size();
}


void ComputeGroundOrientedBoundingBox(const pcl::PointCloud<PointT>::Ptr& cloud, OBBox& box, const int ground)
{
    if (cloud == nullptr)
    {
        ROS_ERROR("Null input cloud pointer!");
        return;
    }
    pcl::PointCloud<PointTFull>::Ptr full_cloud (new pcl::PointCloud<PointTFull>);
    pcl::copyPointCloud(*cloud, *full_cloud);
    ComputeGroundOrientedBoundingBox(full_cloud, box, ground);
}


void ComputeGroundOrientedBoundingBox(const pcl::PointCloud<PointTFull>::Ptr& cloud, OBBox& box, const int ground)
{
    if (cloud == nullptr)
    {
        ROS_ERROR("Null input cloud pointer!");
        return;
    }

    ApproxMVBB::Matrix2Dyn points(2, cloud->points.size());  // x, y in approxMVBB
    
    for (size_t i = 0u; i < cloud->points.size(); ++i) 
    {
        Eigen::Vector3d point (cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        points(0, i) = point((ground+1)%3);
        points(1, i) = point((ground+2)%3);
    }

    ApproxMVBB::MinAreaRectangle min_rectangle(points);
    min_rectangle.compute();
    ApproxMVBB::MinAreaRectangle::Box2d box_2d = min_rectangle.getMinRectangle();

    Eigen::Vector2d pos_2d = box_2d.m_p + box_2d.m_v * box_2d.m_vL/2 + box_2d.m_u * box_2d.m_uL/2;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::getMinMax3D(*cloud, min, max);

    Eigen::Matrix3f rotation;
    if (ground == 0)
        rotation << 1, 0,             0,
                    0, box_2d.m_u(0), box_2d.m_v(0),
                    0, box_2d.m_u(1), box_2d.m_v(1);
    else if (ground == 1)
        rotation << box_2d.m_v(1), 0, box_2d.m_u(1),
                    0,             1, 0,
                    box_2d.m_v(0), 0, box_2d.m_u(0);
    else if (ground == 2)
        rotation << box_2d.m_u(0), box_2d.m_v(0), 0,
                    box_2d.m_u(1), box_2d.m_v(1), 0,
                    0,             0,             1;

    box.pos(ground) = (min(ground) + max(ground))/2;
    box.pos((ground+1)%3) = pos_2d(0);
    box.pos((ground+2)%3) = pos_2d(1);

    box.aligned_dims(ground) = max(ground) - min(ground);
    box.aligned_dims((ground+1)%3) = box_2d.m_uL;
    box.aligned_dims((ground+2)%3) = box_2d.m_vL;

    box.quat = Eigen::Quaternionf(rotation);
}


float ComputeBoxPlanePenetration (const OBBox& box, const Eigen::Vector4f& plane)
{
    Eigen::Matrix4f transform = GetHomogeneousTransformMatrix (box.pos, box.quat, 1.0);
    Eigen::MatrixXf corners (4, 8);
    corners << box.aligned_dims(0)/2, box.aligned_dims(0)/2, box.aligned_dims(0)/2, box.aligned_dims(0)/2, -box.aligned_dims(0)/2, -box.aligned_dims(0)/2, -box.aligned_dims(0)/2, -box.aligned_dims(0)/2,
               box.aligned_dims(1)/2, box.aligned_dims(1)/2, -box.aligned_dims(1)/2, -box.aligned_dims(1)/2, box.aligned_dims(1)/2, box.aligned_dims(1)/2, -box.aligned_dims(1)/2, -box.aligned_dims(1)/2,
               box.aligned_dims(2)/2, -box.aligned_dims(2)/2, box.aligned_dims(2)/2, -box.aligned_dims(2)/2, box.aligned_dims(2)/2, -box.aligned_dims(2)/2, box.aligned_dims(2)/2, -box.aligned_dims(2)/2,
               1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    corners = transform * corners;
     
    Eigen::VectorXf distance_array = corners.transpose() * plane;
    return std::min(distance_array.minCoeff(), 0.0f);
}

}
