#ifndef MAP_PROCESSING_NODE_H_
#define MAP_PROCESSING_NODE_H_

#include "common.h"
#include "io.h"
#include "visualization.h"
#include "object_cad.h"
#include "collision_check.h"

namespace MapProcessing
{

extern Obj3D::Ptr wall;

class MapProcessingNode
{

public:
    MapProcessingNode(ros::NodeHandle& node_handle);
    ~MapProcessingNode(){};

    void Run();
    void LoadObjectDatabase(std::unordered_map<std::string, std::vector<ObjCAD::Ptr>>& cad_database);
    void LoadInstanceSegments(const std::unordered_map<std::string, std::vector<ObjCAD::Ptr>>& cad_database, std::vector<Obj3D::Ptr>& objects);
    void LoadGroundTruthSegments(std::unordered_map<int, OBBox>& gt_objects);
    void LoadGroundTruthSegments(std::vector<Obj3D::Ptr>& objects);

    void DecideSupportingParents (const std::vector<Obj3D::Ptr>& objects, std::unordered_map<Obj3D::Ptr, std::unordered_map<Obj3D::Ptr, Eigen::Vector4f>>& parent_child_map,
                                                            std::queue<Obj3D::Ptr>& obj_to_check);
    void BuildContactGraph (const std::unordered_map<Obj3D::Ptr, std::unordered_map<Obj3D::Ptr, Eigen::Vector4f>>& parent_child_map,
                                                            const std::unordered_map<int, OBBox>& gt_objects, std::queue<Obj3D::Ptr>& obj_to_check, pgm::ParseGraphMap::Ptr& contact_graph);
    void ComputeParentCandidates (const std::vector<Obj3D::Ptr>& objects, std::unordered_map<Obj3D::Ptr, std::vector<Obj3D::Ptr>>& candidates);
    void ComputeSupportingScores (const Obj3D::Ptr& child, const Obj3D::Ptr& parent, std::vector<std::pair<Eigen::Vector4f, float>>& scores);
    void UpdateObjectsViaSupportingRelations (std::vector<Obj3D::Ptr>& objects, std::unordered_map<Obj3D::Ptr, std::unordered_map<Obj3D::Ptr, Eigen::Vector4f>>& parent_child_map,
                                                            const std::queue<Obj3D::Ptr>& obj_to_check);


    void MatchCADToSegmentsCoarse (const std::vector<Obj3D::Ptr>& objects, const std::unordered_map<std::string, std::vector<ObjCAD::Ptr>>& cad_database,
                                                            std::unordered_map<Obj3D::Ptr, std::vector<ObjCADCandidate::Ptr>>& cad_candidates_map);
    void ComputeDimsMatchingError (const Eigen::Vector3f object_normalized_dims, const ObjCAD::Ptr& cad, std::unordered_map<int, float>& matching_errors);
    void ComputeSupportingPlaneMatchingError (const std::vector<std::pair<float, Eigen::Vector4f>>& supporting_planes, const ObjCAD::Ptr& cad, std::unordered_map<int, float>& matching_errors,
                                                            std::unordered_map<int, std::vector<int>>& pose_supporting_plane_map);
    void ComputePlaneMatchingError (const std::vector<std::vector<Eigen::Vector4f>>& canonical_transformed_planes, const ObjCAD::Ptr& cad, 
                                        std::unordered_map<int, float>& matching_errors, std::unordered_map<int, std::vector<int>>& pose_plane_map);
    bool ValidateSupportedArea (const ObjCADCandidate::Ptr& cad_candidate, const float min_valid_area);
    bool ValidateSupportingAffordance (ObjCADCandidate::Ptr& cad_candidate);
    bool CheckSupportingAffordance (const Eigen::Vector4f& supporting_plane, const pcl::PointCloud<PointT>::Ptr& plane_cloud, 
                                            const pcl::PointCloud<PointT>::Ptr& above_plane_cloud, const Obj3D::Ptr& child);
    bool ValidateCollisionFreeCriteria (const Obj3D::Ptr& object, ObjCADCandidate::Ptr& cad_candidate, const std::vector<Obj3D::Ptr>& objects);

    void ComputePotentialCollisionPairs (const std::unordered_map<Obj3D::Ptr, std::vector<ObjCADCandidate::Ptr>>& cad_selected_candidates, 
                                                        std::unordered_map<Obj3D::Ptr, std::unordered_map<Obj3D::Ptr, float>>& possible_collision_map);

    void MatchCADToSegmentsFine (const std::vector<Obj3D::Ptr>& objects, std::unordered_map<Obj3D::Ptr, std::vector<ObjCADCandidate::Ptr>>& cad_selected_candidates);

    float ComputeAlignmentError (const Obj3D::Ptr& object, ObjCADCandidate::Ptr& cad_candidate);

    void GlobalRegulation (const std::vector<Obj3D::Ptr>& objects, std::unordered_map<Obj3D::Ptr, std::vector<ObjCADCandidate::Ptr>>& cad_selected_candidates, 
                                                        const std::unordered_map<Obj3D::Ptr, std::unordered_map<Obj3D::Ptr, float>>& possible_collision_map, std::unordered_map<Obj3D::Ptr, ObjCADCandidate::Ptr>& map_candidate);
    bool ValidateWallConstraint (const std::vector<Eigen::Vector4f>& wall_planes, const ObjCADCandidate::Ptr& cad_candidate);
    void ComputeCollision (const std::unordered_map<Obj3D::Ptr, ObjCADCandidate::Ptr>& map_candidate, const std::unordered_map<Obj3D::Ptr, std::unordered_map<Obj3D::Ptr, float>>& possible_collision_map, 
                                                                std::unordered_map<Obj3D::Ptr, std::vector<Obj3D::Ptr>>& collided_pairs, std::set<Obj3D::Ptr>& collided_objects);
    float RecomputeCollision (const Obj3D::Ptr& object, const ObjCADCandidate::Ptr& cad_candidate, const std::unordered_map<Obj3D::Ptr, ObjCADCandidate::Ptr>& map_candidate, 
                                                const std::unordered_map<Obj3D::Ptr, float>& possible_collided_objects, std::vector<Obj3D::Ptr>& collided_objects);
    
    void FillContactGraph (const std::vector<Obj3D::Ptr>& objects, const std::unordered_map<int, OBBox>& gt_objects, std::unordered_map<Obj3D::Ptr, ObjCADCandidate::Ptr>& map_candidate, 
                                            pgm::ParseGraphMap::Ptr& contact_graph);

    Eigen::Matrix4f GetRefineTransform (const Eigen::Vector4f& param, const float object_ground_dim, const float cad_ground_dim);


private:
    ros::NodeHandle node_handle_;

    std::string output_folder_;
    std::string instance_segments_path_;
    std::string instance_id_file_;
    std::string cad_id_file_;
    std::string ground_truth_prefix_;

    Viewer::Ptr viewer;
    bool visualize_scene_mesh_;
    bool visualize_optimized_alignment_;
    bool visualize_plane_estimation_;
    bool visualize_global_regulation_;

    bool if_verbose_;
    bool save_contact_graph_;
    bool match_ground_truth_;
    bool match_cad_;
    bool match_cabinet_;
    bool filter_small_objects_;
    bool write_refined_bbox_;
    float scale_ratio_;
    int k_cad_candidates_;
    float wall_heuristics_;
    float lambda_;

};

}



#endif