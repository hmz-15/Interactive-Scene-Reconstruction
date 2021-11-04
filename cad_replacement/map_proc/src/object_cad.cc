#include "object_cad.h"


namespace MapProcessing
{

// 24 canonical poses
std::vector<Eigen::Matrix4f> canonical_base_transforms;

// The axis of the upright direction
Eigen::Vector3f ground_axis;
int ground;

// The path of cad database
std::string cad_database_path;

// Layout classes
const std::vector<std::string> layout_class = {"Background", "Floor", "Wall", "Ceiling"};


void Obj3D::ComputeBox() 
{
    ComputeGroundOrientedBoundingBox(cloud, box, ground);
    diameter = sqrt(box.aligned_dims.transpose() * box.aligned_dims);
    bottom_height = (box.pos.transpose()*ground_axis - box.aligned_dims.transpose()*ground_axis/2)(0);
    top_height = (box.pos.transpose()*ground_axis + box.aligned_dims.transpose()*ground_axis/2)(0);
}


Eigen::MatrixXf Obj3D::GetBoxCorners4D ()
{
    Eigen::Matrix4f transform = GetHomogeneousTransformMatrix (box.pos, box.quat);
    Eigen::Vector3f aligned_dims = box.aligned_dims;

    // 8 box corners in local and global frames
    Eigen::MatrixXf corners (4, 8);
    corners << aligned_dims(0)/2, aligned_dims(0)/2, aligned_dims(0)/2, aligned_dims(0)/2, -aligned_dims(0)/2, -aligned_dims(0)/2, -aligned_dims(0)/2, -aligned_dims(0)/2,
               aligned_dims(1)/2, aligned_dims(1)/2, -aligned_dims(1)/2, -aligned_dims(1)/2, aligned_dims(1)/2, aligned_dims(1)/2, -aligned_dims(1)/2, -aligned_dims(1)/2,
               aligned_dims(2)/2, -aligned_dims(2)/2, aligned_dims(2)/2, -aligned_dims(2)/2, aligned_dims(2)/2, -aligned_dims(2)/2, aligned_dims(2)/2, -aligned_dims(2)/2,
               1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    corners = transform * corners;
    
    return corners;
}


void Obj3D::ComputePlanes()
{        
    planes.clear();
    float distance_threshood = 0.03;
    if (box.aligned_dims.minCoeff()/4 < distance_threshood)
        distance_threshood = box.aligned_dims.minCoeff()/4;
    ComputePlanesRANSAC(cloud, planes, distance_threshood);
}


void Obj3D::ComputePotentialSupportingPlanes()
{
    potential_supporting_planes.clear();
    for (int i = 0; i < planes.size(); i++)
        if (IsSupportingPlane(planes[i], ground_axis, 0.9))
            potential_supporting_planes.push_back(planes[i]);
}


void Obj3D::ComputeSupportDistance(float child_bottom_height, std::vector<std::pair<Eigen::Vector4f, float>>& distances)
{
    for (int i = 0; i < potential_supporting_planes.size(); i++)
    {
        float dist = std::abs(child_bottom_height + potential_supporting_planes[i](3));
        distances.push_back(std::make_pair(potential_supporting_planes[i], dist));
    }
    // Hidden planes
    if (potential_supporting_planes.size() == 0)
    {
        Eigen::Vector4f hidden_plane_1 (ground_axis(0), ground_axis(1), ground_axis(2), -child_bottom_height);
        // Eigen::Vector4f hidden_plane_2 (ground_axis(0), ground_axis(1), ground_axis(2), -top_height);
        // distances.push_back(std::make_pair(hidden_plane_1, 0.0f));
        // distances.push_back(std::make_pair(hidden_plane_2, std::abs(-top_height+child_bottom_height)));
        distances.push_back(std::make_pair(hidden_plane_1, std::min(std::abs(-top_height+child_bottom_height), std::abs(-bottom_height+child_bottom_height))));
    }
}


void Obj3D::RefineAsSupportingChild() 
{
    if (supporting_parent.first == nullptr)
    {
        Eigen::Vector4f plane (0.0, 0.0, 0.0, 0.0);
        plane.head(3) = ground_axis;
        supporting_parent = std::make_pair(nullptr, plane);
    }
    float plane_height = - supporting_parent.second(3);
    box.aligned_dims(ground) += (bottom_height - plane_height);
    box.pos(ground) -= (bottom_height - plane_height)/2;
    diameter = sqrt(box.aligned_dims.transpose() * box.aligned_dims);
    bottom_height = plane_height;
}

void Obj3D::UpdateAsSupportingParent(Obj3D::Ptr child, Eigen::Vector4f supporting_plane) 
{
    float plane_height = - supporting_plane(3);
    float supporting_plane_height_ratio = (plane_height - bottom_height)/box.aligned_dims(ground);
    auto it = std::find_if(supporting_planes.begin(), supporting_planes.end(), [=] (const auto& f) 
                                                    { return (std::abs(f.first-supporting_plane_height_ratio) < 0.05);});
    if (it != supporting_planes.end())
        supporting_children.insert(std::make_pair(child, it-supporting_planes.begin()));  
    else
    {
        supporting_planes.push_back(std::make_pair(supporting_plane_height_ratio, supporting_plane));
        supporting_children.insert(std::make_pair(child, supporting_planes.size()-1));  
    }
}


void Obj3D::UpdatePlanesViaSupporting()
{
    for (auto plane_it = planes.begin(); plane_it != planes.end(); )
    {
        if (IsSupportingPlane (*plane_it, ground_axis))
        {
            auto support_it = std::find_if(supporting_planes.begin(), supporting_planes.end(), [=] (const auto& f) 
                                                                                    {return (std::abs(f.second(3)-(*plane_it)(3)) < 0.01);});
            if (support_it != supporting_planes.end())
            {
                planes.erase(plane_it);
                continue;
            }
        }
        plane_it++;
    }
} 


Eigen::Matrix4f ObjCADCandidate::GetTransform (bool with_scale)
{
    OBBox box = object->GetBox();
    Eigen::Matrix4f transform_matrix;
    if (with_scale)
        transform_matrix = GetHomogeneousTransformMatrix (box.pos, box.quat, scale); // world to scaled box
    else
        transform_matrix = GetHomogeneousTransformMatrix (box.pos, box.quat); // world to box

    transform_matrix = transform_matrix * refine_transform * canonical_base_transforms[pose_index]; // world to aligned CAD

    if (set_absolute_height)
        transform_matrix(ground, 3) = absolute_height;

    return transform_matrix;
}


OBBox ObjCADCandidate::GetAlignedBox ()
{
    pcl::PointCloud<PointT>::Ptr cloud = GetTransformedSampledCloudPtr();
    OBBox box;
    ComputeGroundOrientedBoundingBox(cloud, box, ground);
    return box;
}


Eigen::MatrixXf ObjCADCandidate::GetAlignedBoxCorners4D ()
{
    Eigen::Matrix4f transform = GetTransform();
    Eigen::Vector3f aligned_dims = GetCADPtr()->GetDims();

    // 8 box corners in local and global frames
    Eigen::MatrixXf corners (4, 8);
    corners << aligned_dims(0)/2, aligned_dims(0)/2, aligned_dims(0)/2, aligned_dims(0)/2, -aligned_dims(0)/2, -aligned_dims(0)/2, -aligned_dims(0)/2, -aligned_dims(0)/2,
               aligned_dims(1)/2, aligned_dims(1)/2, -aligned_dims(1)/2, -aligned_dims(1)/2, aligned_dims(1)/2, aligned_dims(1)/2, -aligned_dims(1)/2, -aligned_dims(1)/2,
               aligned_dims(2)/2, -aligned_dims(2)/2, aligned_dims(2)/2, -aligned_dims(2)/2, aligned_dims(2)/2, -aligned_dims(2)/2, aligned_dims(2)/2, -aligned_dims(2)/2,
               1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    corners = transform * corners;
    
    return corners;
}


pcl::PolygonMesh::Ptr ObjCADCandidate::GetTransformedMeshPtr ()
{
    pcl::PolygonMesh::Ptr cad_mesh = cad_candidate->GetMeshPtr();

    if (transformed_mesh == nullptr)
        transformed_mesh.reset(new pcl::PolygonMesh);

    Eigen::Matrix4f current_transform = GetTransform ();
    TransformMesh (cad_mesh, current_transform, transformed_mesh);

    return transformed_mesh;
}


pcl::PointCloud<PointT>::Ptr ObjCADCandidate::GetTransformedSampledCloudPtr ()
{
    pcl::PointCloud<PointT>::Ptr sampled_cloud = cad_candidate->GetSampledCloudPtr();

    if (transformed_sample_cloud == nullptr)
        transformed_sample_cloud.reset(new pcl::PointCloud<PointT>);

    Eigen::Matrix4f current_transform = GetTransform ();
    pcl::transformPointCloud(*sampled_cloud, *transformed_sample_cloud, current_transform);

    return transformed_sample_cloud;
}


}
