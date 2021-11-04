#ifndef OBJECT_H_
#define OBJECT_H_

#include "common.h"
#include "io.h"
#include "utils.h"
#include "3rd_party/mesh_sampling.h"


namespace MapProcessing
{

extern std::vector<Eigen::Matrix4f> canonical_base_transforms;
extern std::string cad_database_path;
extern Eigen::Vector3f ground_axis;
extern int ground;
extern const std::vector<std::string> layout_class;

// Volumetric scene entities
class Obj3D{
public:
    using Ptr = std::shared_ptr<Obj3D>;

    // Constructor
    Obj3D(int ID, std::string category, pcl::PointCloud<PointTFull>::Ptr cloud_input, pcl::PolygonMesh::Ptr mesh_input = nullptr): 
        id(ID), category_name(category), mesh(mesh_input), cloud(cloud_input){};

    // Check if layout class
    bool IsLayout()
    {
        if (std::find(layout_class.begin(), layout_class.end(), category_name) != layout_class.end())
            return true;
        else
            return false;
    }

    // Compute box and related variabvles based on cloud
    void ComputeBox();
    void SetBox(OBBox& box_in) {box = box_in; };

    // Get box corners in generalized coordinate
    Eigen::MatrixXf GetBoxCorners4D ();

    // Estimate planes and potential supporting planes
    void ComputePlanes();
    void ComputePotentialSupportingPlanes();

    // Get private members
    pcl::PolygonMesh::Ptr GetMeshPtr() {return mesh;}
    pcl::PointCloud<PointTFull>::Ptr GetPointCloudPtr() {return cloud;}
    std::vector<Eigen::Vector4f> GetPlanes() {return planes;}
    std::vector<Eigen::Vector4f> GetPotentialSupportingPlanes() {return potential_supporting_planes;}
    std::vector<std::pair<float, Eigen::Vector4f>> GetSupportingPlanes() {return supporting_planes;}

    OBBox GetBox() {return box;}
    float GetDiameter() {return diameter;}
    float GetBottomHeight() {return bottom_height;}
    float GetTopHeight() {return top_height;}  
    std::pair<Obj3D::Ptr, Eigen::Vector4f> GetSupportingParent() {return supporting_parent;}
    std::unordered_map<Obj3D::Ptr, int> GetSupportingChildren() {return supporting_children;}

    // Compute the distance from each potential supporting plane to the given bottom height of a candidate child (gravity aligned)
    void ComputeSupportDistance(float child_bottom_height, std::vector<std::pair<Eigen::Vector4f, float>>& distances);

    // Set supporting parent, refine box as supporting child
    void SetSupportingParent(Obj3D::Ptr parent, Eigen::Vector4f supporting_plane) {supporting_parent = std::make_pair(parent, supporting_plane);}
    void RefineAsSupportingChild();

    // Clear and update supporting information supporting parent
    void ClearInfoAsSupportingParent()
    {
        supporting_children.clear();
        supporting_planes.clear();
    }
    void UpdateAsSupportingParent(Obj3D::Ptr child, Eigen::Vector4f supporting_plane); 

    // Remove supporting planes from plane list (separate for further processing)
    void UpdatePlanesViaSupporting();

    // Get transform in world frame
    Eigen::Matrix4f GetBoxTransform () {return GetHomogeneousTransformMatrix (box.pos, box.quat, 1.0);}

    // Public variables
    int id;
    std::string category_name;


private:   
    OBBox box;    
    std::vector<Eigen::Vector4f> planes; // a,b,c,d
    std::vector<Eigen::Vector4f> potential_supporting_planes; // a,b,c,d
    std::vector<std::pair<float, Eigen::Vector4f>> supporting_planes; // plane_height_ratio , (a,b,c,d)
    std::unordered_map<Obj3D::Ptr, int> supporting_children; // child, index in supporting_planes
    pcl::PolygonMesh::Ptr mesh;
    pcl::PointCloud<PointTFull>::Ptr cloud;
    std::pair<Obj3D::Ptr, Eigen::Vector4f> supporting_parent;

    float bottom_height;
    float top_height;
    float diameter;
};

    
// CAD object in the database
class ObjCAD{
public:
    using Ptr = std::shared_ptr<ObjCAD>;

    // Constructor
    ObjCAD(std::string id, std::string category, std::vector<Eigen::Vector4f>& planes_input, Eigen::Vector3f dims):
        cad_id(id), category_name(category), planes(planes_input), aligned_dims(dims)
    {
        diameter = sqrt(dims.transpose() * dims);
    }

    // Get private variables
    Eigen::Vector3f GetDims() {return aligned_dims;}
    float GetDiameter() {return diameter;}
    std::vector<Eigen::Vector4f> GetPlanes() {return planes;}
    pcl::PolygonMesh::Ptr GetMeshPtr() 
    {
        if (mesh == nullptr)
        {
            mesh.reset (new pcl::PolygonMesh);
            ReadMeshFromOBJ (cad_database_path + "/" + cad_id + ".obj", mesh);      
        }
        return mesh;
    }
    pcl::PointCloud<PointT>::Ptr GetSampledCloudPtr()
    {
        if (sample_cloud == nullptr)
        {
            pcl::PolygonMesh::Ptr mesh = GetMeshPtr();
            sample_cloud.reset(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointTFull>::Ptr sample_full_cloud (new pcl::PointCloud<PointTFull>);
            int num_sample_points = (int)(diameter*diameter*2000);
            SampleMesh (*mesh, sample_full_cloud, num_sample_points, 0.01);
            pcl::copyPointCloud(*sample_full_cloud, *sample_cloud);
        }
        return sample_cloud;
    }

    // Set private variables
    void SetMeshPtr(pcl::PolygonMesh::Ptr mesh_ptr) {mesh = mesh_ptr;}

    std::string cad_id;
    std::string category_name;

private:
    std::vector<Eigen::Vector4f> planes;
    Eigen::Vector3f aligned_dims;
    float diameter;

    pcl::PolygonMesh::Ptr mesh;
    pcl::PointCloud<PointT>::Ptr sample_cloud;
};


// An instantiated CAD as a replacement candidate of a volumetric scene entity
class ObjCADCandidate{
public:
    using Ptr = std::shared_ptr<ObjCADCandidate>;

    // Constructor
    ObjCADCandidate(Obj3D::Ptr obj, ObjCAD::Ptr cad, int pose_id, float matching_error, std::vector<int>& supporting_plane_match, std::vector<int>& plane_match, float match_scale):
                object(obj), cad_candidate(cad), pose_index(pose_id), coarse_matching_error(matching_error), supporting_plane_matching_index(supporting_plane_match),
                plane_matching_index(plane_match), scale(match_scale)
    {
        refine_transform = Eigen::Matrix4f::Identity();
        fine_matching_error = 10;
    }

    // Get private variables
    Obj3D::Ptr GetObjPtr() {return object;}
    ObjCAD::Ptr GetCADPtr() {return cad_candidate;}
    int GetPoseID() {return pose_index;}
    float GetScale() {return scale;}
    float GetCoarseMatchingError() {return coarse_matching_error;}
    float GetFineMatchingError() {return fine_matching_error;}
    std::vector<int> GetSupportingPlaneMatch() {return supporting_plane_matching_index;};
    std::vector<int> GetPlaneMatch() {return plane_matching_index;};
    
    // Get transform in world frame
    Eigen::Matrix4f GetTransform (bool with_scale = true);
    // Get aligned box
    OBBox GetAlignedBox ();
    // Get aligned box corners in generalized coordinate
    Eigen::MatrixXf GetAlignedBoxCorners4D ();
    // Get transformed mesh, sampled cloud
    pcl::PolygonMesh::Ptr GetTransformedMeshPtr ();
    pcl::PointCloud<PointT>::Ptr GetTransformedSampledCloudPtr ();

    // Set private variables
    void SetScale(float s) {scale = s;}
    void SetRefinedTransform(const Eigen::Matrix4f& transform) {refine_transform = transform;}
    void SetFineMatchingError(float e) {fine_matching_error = e;}
    void SetSupportingPlaneMatch(std::vector<int>& match) {supporting_plane_matching_index = match;};

    // Set absolute height
    void SetHeight(float h) {absolute_height = h; set_absolute_height = true;}
    

private:
    Obj3D::Ptr object;
    ObjCAD::Ptr cad_candidate;
    int pose_index;
    float scale;
    std::vector<int> supporting_plane_matching_index;
    std::vector<int> plane_matching_index;

    float coarse_matching_error;
    float fine_matching_error;

    Eigen::Matrix4f refine_transform; // No scale, refined from initalized alignment using OBBOX
    // May need to fix the absolute height
    float absolute_height = 0.0f;
    bool set_absolute_height = false;

    pcl::PolygonMesh::Ptr transformed_mesh;
    pcl::PointCloud<PointT>::Ptr transformed_sample_cloud;

};


}

#endif