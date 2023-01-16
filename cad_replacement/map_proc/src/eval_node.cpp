#include <fstream>
#include <cctype>

#include "eval_node.h"


namespace MapProcessing{

const std::vector<std::string> panoptic_class = {"Floor", "Wall", "Backpack", "Chair", "Table", "TV", "Bed", "Refrigerator", "Cabinet", "Couch"};

const std::vector<std::string> annotated_class = {"floor", "wall", "bag", "chair", "table", "monitor", "bed", "fridge", "cabinet", "sofa", "desk"};

const std::map<int, std::vector<int>> class_mapping = {{0, {0}}, {1, {1}}, {2, {2}}, {3, {3}}, {4, {4,10}}, {5, {5}}, {6, {6}}, {7, {7}}, {8, {8}}, {9, {9}}};


EvalNode::EvalNode(ros::NodeHandle& node_handle): node_handle_(node_handle)
{
    // load rosparam
    node_handle.param<std::vector<std::string>>("sequence_id", sequence_id_list_, {});
    node_handle_.param<std::string>("output_folder", output_folder_, ros::package::getPath("map_proc"));
    node_handle.param<bool>("use_refined_id", use_refined_id_, false);
    node_handle.param<std::string>("GT_annotation_path", GT_annotation_path_, "");
    node_handle.param<std::string>("save_result_file_name", save_result_file_name_, "result.json");
    
    node_handle.param<bool>("if_verbose", if_verbose_, false);
    node_handle.param<bool>("use_nyu_class", use_nyu_class_, false);        

    node_handle.param<bool>("compute_per_class_eval", compute_per_class_eval_, false);
    node_handle.param<int>("ground_axis", ground, 2);
    ground_axis << 0.0, 0.0, 0.0;
    ground_axis(ground) = 1.0;  
}


void EvalNode::LoadPanopticMap(const std::string sequence_id, std::map<std::string, std::vector<Obj3D::Ptr>>& objects, pcl::PointCloud<PointTLabel>::Ptr& map_cloud)
{
    if (map_cloud == nullptr)
    {
        ROS_ERROR("Null input cloud pointer!");
        return;
    }

    std::string instance_segments_path = output_folder_ + "/" + sequence_id + "/panoptic_segments/";
    // std::string instance_segments_path = "/home/muzhi/Workspace/Robot-Vision-System/src/Robot-Vision-System/output/robust_new/" + sequence_id + "/";
    std::string instance_id_file = instance_segments_path + "id.json";
    if(use_refined_id_)
        instance_id_file = instance_segments_path + "refined_box.json";

    std::ifstream infile(instance_id_file);
    nlohmann::json j;
    infile >> j;

    int instance_label;
    std::string semantic_label;

    for (const auto& element: j.items())
    {
        instance_label = std::stoi(element.key());

        pcl::PointCloud<PointTFull>::Ptr cloud (new pcl::PointCloud<PointTFull>);
        pcl::PointCloud<PointTLabel>::Ptr cloud_xyzl (new pcl::PointCloud<PointTLabel>);
        ReadCloudFromPLY (instance_segments_path + std::to_string(instance_label) + ".ply", cloud, false); 
        pcl::copyPointCloud (*cloud, *cloud_xyzl);
        *map_cloud += *cloud_xyzl;

        semantic_label = element.value(); 

        if (std::find(panoptic_class.begin(), panoptic_class.end(), semantic_label) != panoptic_class.end())
        {
            // Remove small segments
            if (cloud_xyzl->points.size() < 1000)
                continue;

            // Process instance segments
            Obj3D::Ptr object;
            if (use_refined_id_)
            {
                semantic_label = element.value()["semantic_class"];
                std::vector<float> box = element.value()["refined_box"];
                Eigen::Vector3f pos(box[0], box[1], box[2]);
                Eigen::Vector3f aligned_dims(box[3], box[4], box[5]); // x,y,z in world frame
                Eigen::Quaternionf quat(box[9], box[6], box[7], box[8]); // w,x,y,z  
                OBBox object_box = {pos, aligned_dims, quat};

                object = std::make_shared<Obj3D> (instance_label, semantic_label, cloud); 
                object->SetBox(object_box);
            }
            else
            {
                // semantic_label = element.value();
                object = std::make_shared<Obj3D> (instance_label, semantic_label, cloud);
                object->ComputeBox();
            }

            if (if_verbose_)
                std::cout << "semantic label " << semantic_label << std::endl;

            auto cate_it = objects.find(semantic_label);
            if (cate_it != objects.end())
                cate_it->second.push_back(object);
            else
            {
                std::vector<Obj3D::Ptr> vect = {object};
                objects.insert(std::make_pair(semantic_label, vect));
            }
        }
    }
}


void EvalNode::ProcessGTMap(const std::string sequence_id, std::map<std::string, std::vector<Obj3D::Ptr>>& objects, pcl::PointCloud<PointTLabel>::Ptr& map_cloud)
{
    // As some sequences don't have nyu_class annotations
    bool nyu_exist = ifDirectoryExist(GT_annotation_path_+"/nyu_class/"+sequence_id);
    std::string current_GT_annotation_path_ = GT_annotation_path_;
    if (use_nyu_class_ && nyu_exist)
        current_GT_annotation_path_ += "/nyu_class";   

    pcl::PointCloud<PointTLabel>::Ptr GT_map_cloud (new pcl::PointCloud<PointTLabel>);
    std::cout << current_GT_annotation_path_ + "/" + sequence_id + "/" + sequence_id + ".ply" <<std::endl;
    ReadCloudFromPLY (current_GT_annotation_path_ + "/" + sequence_id + "/" + sequence_id + ".ply", GT_map_cloud); 
        
    pcl::KdTreeFLANN<PointTLabel> kdtree;
    kdtree.setInputCloud (GT_map_cloud);
    for (int i = 0; i < map_cloud->points.size(); i++)
    {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if (kdtree.nearestKSearch (map_cloud->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            map_cloud->points[i].label = GT_map_cloud->points[pointIdxNKNSearch[0]].label;
            // std::cout << map_cloud->points[i].label <<std::endl;
        }
    }

    TiXmlDocument doc(current_GT_annotation_path_ + "/" + sequence_id + "/" + sequence_id + ".xml");
    bool loadOkay = doc.LoadFile();
	if (loadOkay)
    {
        TiXmlElement *annotation = doc.FirstChildElement("annotation");
        // TiXmlNode *label = annotation->FirstChild("label");
        TiXmlNode *label = 0;
        int i = 0;
        while (label = annotation->IterateChildren(label))
        {
            std::string str = label->ToElement()->Attribute("id");
            std::string text_str = label->ToElement()->Attribute("text");
            std::string class_str = text_str;
            if (use_nyu_class_ && nyu_exist)
                if (label->ToElement()->Attribute("nyu_class"))
                    std::string class_str = label->ToElement()->Attribute("nyu_class");            

            auto equal = [=] (const std::string& s) {return (class_str.find(s)!=std::string::npos || text_str.find(s)!=std::string::npos);};
            auto it = std::find_if(annotated_class.begin(), annotated_class.end(), equal);
            if (it != annotated_class.end())
            {
                std::string category_name = *it;
                int id = std::stoi(str);
                // Convert to panoptic class
                int annot_index = it - annotated_class.begin();
                for (const auto& class_match: class_mapping)
                {
                    if (std::find(class_match.second.begin(), class_match.second.end(), annot_index) != class_match.second.end())
                    {
                        category_name = panoptic_class[class_match.first];
                        break;
                    }
                }

                pcl::PointCloud<PointTLabel>::Ptr label_cloud (new pcl::PointCloud<PointTLabel>);
                ExtractCloudByLabel (map_cloud, label_cloud, id);
                pcl::PointCloud<PointTFull>::Ptr full_cloud (new pcl::PointCloud<PointTFull>);
                pcl::copyPointCloud(*label_cloud, *full_cloud);

                Obj3D::Ptr object = std::make_shared<Obj3D> (id, category_name, full_cloud);
                object->ComputeBox();

                auto cate_it = objects.find(object->category_name);
                if (cate_it != objects.end())
                {
                    if(object->category_name == "Floor" || object->category_name == "Wall" || object->category_name == "Ceiling")
                    {
                        *(cate_it->second[0]->GetPointCloudPtr()) += *full_cloud;
                        for (int index = 0; index < map_cloud->points.size(); index++)
                        {
                            if(map_cloud->points[index].label == object->id)
                                map_cloud->points[index].label == cate_it->second[0]->id;
                        }    
                        continue;   
                    }
                    else
                        cate_it->second.push_back(object);
                }
                else
                {
                    std::vector<Obj3D::Ptr> vect = {object};
                    objects.insert(std::make_pair(object->category_name, vect));
                }

                if (if_verbose_)
                    std::cout << "semantic label " << object->category_name << std::endl;
            }
        }
    }
}


void EvalNode::ComputeMatch (const std::map<std::string, std::vector<Obj3D::Ptr>>& map_objects, const std::map<std::string, std::vector<Obj3D::Ptr>>& GT_objects, 
                            const pcl::PointCloud<PointTLabel>::Ptr& map_cloud, std::unordered_map<std::string, PanoInfo>& pano_match, 
                            std::unordered_map<std::string, InstInfo>& inst_match, std::unordered_map<std::string, InstInfo>& box_match)
{
    pcl::KdTreeFLANN<PointTLabel> kdtree;
    kdtree.setInputCloud (map_cloud);
    
    for (auto cate_it = map_objects.begin(); cate_it != map_objects.end(); cate_it++)
    {
        std::string map_category = cate_it->first;
        std::vector<Obj3D::Ptr> map_objects_vect = cate_it->second;
        auto pano_class_it = std::find(panoptic_class.begin(), panoptic_class.end(), map_category);
        if (pano_class_it == panoptic_class.end())
            ROS_ERROR("Invalid map entity class!");   

        // Find same class in ground truth
        std::vector<Obj3D::Ptr> gt_objects_vect;
        auto gt_cate_it = GT_objects.find(map_category);
        if (gt_cate_it != GT_objects.end())
            gt_objects_vect = gt_cate_it->second; 

        // Initialize pano_match, inst_match and box_match
        if (pano_match.find(map_category) == pano_match.end())
        {
            PanoInfo new_info = PanoInfo(0.0f, 0.0f, (float)map_objects_vect.size(), (float)gt_objects_vect.size()); // Initialize FP. FN as # of predictions / gt
            pano_match.insert(std::make_pair(map_category, new_info));  
        }
        // Only for thing class
        if (map_category != "Floor" && map_category != "Wall" && map_category != "Ceiling")
        {
            if (inst_match.find(map_category) == inst_match.end())
            {
                InstInfo new_info = InstInfo((float)gt_objects_vect.size()); // initialize number of gt_objects
                inst_match.insert(std::make_pair(map_category, new_info));
            }

            if (box_match.find(map_category) == box_match.end())
            {
                InstInfo new_info = InstInfo((float)gt_objects_vect.size()); // initialize number of gt_objects
                box_match.insert(std::make_pair(map_category, new_info));
            }
        }          

        // Loop over map objects in the class
        std::unordered_set<int> assigned_GT_id;
        std::unordered_set<int> box_assigned_GT_id;
        for (int i = 0; i < map_objects_vect.size(); i++)
        {
            // Determine GT object candidate by overlap
            std::vector<std::pair<int, int>> candidates; // (id, count)
            pcl::PointCloud<PointTLabel>::Ptr label_cloud (new pcl::PointCloud<PointTLabel>);
            pcl::copyPointCloud(*map_objects_vect[i]->GetPointCloudPtr(), *label_cloud);
            for (int j = 0; j < label_cloud->points.size(); j++)
            {  
                std::vector<int> pointIdxNKNSearch(1);
                std::vector<float> pointNKNSquaredDistance(1);
                if (kdtree.nearestKSearch (label_cloud->points[j], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                {
                    int annot_id = map_cloud->points[pointIdxNKNSearch[0]].label;
                    auto equ_id = [annot_id](const auto& a) {return a.first == annot_id;};
                    auto it = std::find_if(candidates.begin(), candidates.end(), equ_id);
                    if (it != candidates.end())
                        it->second ++;
                    else
                        candidates.push_back(std::make_pair(annot_id, 1));
                }
            }
            // Sort candidates based on the number of counts
            auto pair_cmp = [](const auto& a, const auto& b) ->bool {return a.second > b.second;};
            std::sort(candidates.begin(), candidates.end(),pair_cmp);


            // Loop over candidates to compute iou and validate class match
            bool match_flag = false;
            for (const auto& candidate: candidates)
            {
                int annot_id = candidate.first;
                int count = candidate.second;
                auto gt_obj_it = std::find_if(gt_objects_vect.begin(), gt_objects_vect.end(), [=](const Obj3D::Ptr& o){return o->id == annot_id;});
                if (gt_obj_it != gt_objects_vect.end())
                {
                    // If matched id, compute iou
                    float iou = (float)count/(float)(label_cloud->points.size() + (*gt_obj_it)->GetPointCloudPtr()->points.size() - count);

                    // Save into panoptic match
                    if (iou > 0.5 && assigned_GT_id.find(annot_id) == assigned_GT_id.end())  // Check whether ground truth assigned
                    {
                        if (if_verbose_)
                            std::cout << map_category << " " <<iou <<std::endl;
                        auto cate_itt = pano_match.find(map_category);
                        if (cate_itt != pano_match.end())
                            cate_itt->second.add_prediction(iou); // add prediction  
                        else
                            std::cout << "Should have initialized pano_match" <<std::endl;

                        // Save into instance (thing class)
                        if (map_category != "Floor" && map_category != "Wall" && map_category != "Ceiling")
                        {
                            auto cate_itt = inst_match.find(map_category);
                            if (cate_itt == inst_match.end())
                                std::cout << "Should have initialized inst_match" <<std::endl;
                            cate_itt->second.add_prediction(label_cloud->points.size(), true); // add positive detection
                        }
                        assigned_GT_id.insert(candidate.first); // Add to assigned  
                        match_flag = true;
                        break;                     
                    }
                }                    
            }
            // Gone through all candidates but with no match
            // Save into instance match
            if (!match_flag)
            {
                if (map_category != "Floor" && map_category != "Wall" && map_category != "Ceiling")
                {
                    auto cate_itt = inst_match.find(map_category);
                    if (cate_itt == inst_match.end())
                        std::cout << "Should have initialized inst_match" <<std::endl;
                    cate_itt->second.add_prediction(label_cloud->points.size(), false); // add negative detection
                }
            }


            // Loop over candidates to compute box iou and validate class match (for thing class)
            if (map_category != "Floor" && map_category != "Wall" && map_category != "Ceiling")
            {
                bool box_match_flag = false;
                for (const auto& candidate: candidates)
                {
                    int annot_id = candidate.first;
                    int count = candidate.second;
                    auto gt_cate_it = GT_objects.find(map_category);

                    // Find same class in ground truth
                    auto gt_obj_it = std::find_if(gt_objects_vect.begin(), gt_objects_vect.end(), [=](const Obj3D::Ptr& o){return o->id == annot_id;});
                    if (gt_obj_it != gt_objects_vect.end())
                    {
                        // Save into box match
                        float box_iou = ComputeGroundedBoxIOU3D (map_objects_vect[i]->GetBox(), (*gt_obj_it)->GetBox(), ground);
                    
                        if (box_iou > 0.5 && box_assigned_GT_id.find(annot_id) == box_assigned_GT_id.end())
                        {
                            auto box_cate_itt = box_match.find(map_category);
                            if (box_cate_itt == box_match.end())
                                std::cout << "Should have initialized box_match" <<std::endl;
                            box_cate_itt->second.add_prediction(label_cloud->points.size(), true); // add positive detection
                            box_assigned_GT_id.insert(candidate.first); // Add to assigned  
                            box_match_flag = true; // Set match flag
                            break;       
                        }
                    }                    
                }
                // Gone through all candidates but with no match
                // Save into box match
                if (!box_match_flag)
                {
                    auto cate_itt = box_match.find(map_category);
                    if (cate_itt == box_match.end())
                        std::cout << "Should have initialized box_match" <<std::endl;
                    cate_itt->second.add_prediction(label_cloud->points.size(), false); // add negative detection
                }
            }
        }         
    }
}


nlohmann::json EvalNode::RunOnce(const std::string sequence_id, std::unordered_map<std::string, PanoInfo>& all_pano_matches, 
                                std::unordered_map<std::string, InstInfo>& all_inst_matches,  std::unordered_map<std::string, InstInfo>& all_box_matches)
{
    std:: cout << "Processing sequence: " + sequence_id << std::endl;
    std::map<std::string, std::vector<Obj3D::Ptr>> map_objects;
    std::map<std::string, std::vector<Obj3D::Ptr>> GT_objects;
    pcl::PointCloud<PointTLabel>::Ptr map_cloud (new pcl::PointCloud<PointTLabel>);
    LoadPanopticMap(sequence_id, map_objects, map_cloud);

    ProcessGTMap(sequence_id, GT_objects, map_cloud);

    std::unordered_map<std::string, PanoInfo> pano_matches;
    std::unordered_map<std::string, InstInfo> inst_matches, box_matches;
    ComputeMatch (map_objects, GT_objects, map_cloud, pano_matches, inst_matches, box_matches);

    nlohmann::json output_json_once;
    /**************** 3D panoptic segmentation **************/
    if (if_verbose_)
        std::cout << "------- 3D panoptic segmentation -------" << std::endl;

    float mPQ = 0.0f;
    float mRQ = 0.0f;
    float mSQ = 0.0f;
    for (const auto& cate_pano: pano_matches)
    {   
        if (if_verbose_)
        {
            std::cout << "category: " << cate_pano.first << std::endl;        
            std::cout << "iou_sum: " << cate_pano.second.get_iou() << std::endl;

            std::vector<float> res (3);
            cate_pano.second.get_TP_FP_FN(res);
            std::cout << "FP" << res[0] << std::endl;
            std::cout << "FP" << res[1] << std::endl;
            std::cout << "FN" << res[2] << std::endl;
        }

        // Store the per-sequence information
        if (compute_per_class_eval_)
        {
            auto all_pano_it = all_pano_matches.find(cate_pano.first);
            if (all_pano_it != all_pano_matches.end())
                all_pano_it->second.add(cate_pano.second);
            else
                all_pano_matches.insert(std::make_pair(cate_pano.first, cate_pano.second));
        }
        
        std::vector<float> res (3);
        cate_pano.second.compute_eval (res);
        float PQ = res[0];
        float SQ = res[1];
        float RQ = res[2];
        output_json_once["pano"][cate_pano.first] = { {"PQ", PQ * 100}, {"RQ", RQ * 100}, {"SQ", SQ * 100} };

        mPQ += PQ;
        mRQ += RQ;
        mSQ += SQ;

        if (if_verbose_)
        {
            // Per-class result on the sequence
            std::cout << "PQ: " << PQ * 100 <<std::endl;
            std::cout << "SQ: " << SQ * 100 <<std::endl;
            std::cout << "RQ: " << RQ * 100 <<std::endl;
        }
    }

    // Class-averaged result
    mPQ /= (float)pano_matches.size();
    mRQ /= (float)pano_matches.size();
    mSQ /= (float)pano_matches.size();

    output_json_once["pano"]["average"] = { {"mPQ", mPQ * 100}, {"mRQ", mRQ * 100}, {"mSQ", mSQ * 100} };

    std::cout << "mPQ: " << mPQ * 100 <<std::endl;
    std::cout << "mSQ: " << mSQ * 100 <<std::endl;
    std::cout << "mRQ: " << mRQ * 100 <<std::endl;
    


    /**************** 3D instance segmentation **************/
    if (if_verbose_)
        std::cout << "------- 3D instance segmentation -------" << std::endl;

    float mAP = 0.0f;
    for (auto& cate_inst: inst_matches)
    {
        // Store the per-sequence information
        if (compute_per_class_eval_)
        {
            auto all_inst_it = all_inst_matches.find(cate_inst.first);
            if (all_inst_it != all_inst_matches.end())
                all_inst_it->second.add(cate_inst.second);
            else
                all_inst_matches.insert(std::make_pair(cate_inst.first, cate_inst.second));
        }

        float current_ap = cate_inst.second.compute_AP();
        output_json_once["inst"][cate_inst.first] = { {"AP", current_ap * 100} };

        mAP += current_ap;

        if (if_verbose_)
        {
            std::cout << "category: " << cate_inst.first << std::endl;
            std::cout << "AP: " << current_ap * 100 <<std::endl;
        }
    }
    mAP /= (float)inst_matches.size();
    output_json_once["inst"]["average"] = { {"mAP", mAP * 100} };
    std::cout << "mAP: " << mAP * 100 << std::endl;
    


    /**************** 3D box estimation **************/
    if (if_verbose_)
        std::cout << "-------3D box estimation -------" << std::endl;

    float box_mAP = 0.0f;
    for (auto& cate_box: box_matches)
    {
        // Store the per-sequence information
        if (compute_per_class_eval_)
        {
            auto all_box_it = all_box_matches.find(cate_box.first);
            if (all_box_it != all_box_matches.end())
                all_box_it->second.add(cate_box.second);
            else
                all_box_matches.insert(std::make_pair(cate_box.first, cate_box.second));
        }

        float current_ap = cate_box.second.compute_AP();
        output_json_once["bbox"][cate_box.first] = { {"AP", current_ap * 100} };

        box_mAP += current_ap;

        if (if_verbose_)
        {
            std::cout << "category: " << cate_box.first << std::endl;
            std::cout << "AP: " << current_ap * 100 <<std::endl;
        }
    }
    box_mAP /= (float)box_matches.size();
    output_json_once["bbox"]["average"] = { {"mAP", box_mAP * 100} };
    std::cout << "mAP: " << box_mAP * 100 << std::endl;

    return output_json_once;

}


void EvalNode::Run()
{
    std::unordered_map<std::string, PanoInfo> all_pano_matches;
    std::unordered_map<std::string, InstInfo> all_inst_matches, all_box_matches;
    nlohmann::json output_json;

    for (auto& sequence_id: sequence_id_list_)
    {
        output_json[sequence_id] = RunOnce(sequence_id, all_pano_matches, all_inst_matches, all_box_matches);
    }

    if (compute_per_class_eval_)
    {
        /**************** 3D panoptic segmentation **************/
        std::cout << "-------------------------- Per-class results--------------------------" << std::endl;
        std::cout << "------- 3D panoptic segmentation -------" << std::endl;

        float mPQ = 0.0f;
        float mRQ = 0.0f;
        float mSQ = 0.0f;
        for (const auto& cate_pano: all_pano_matches)
        {
            std::cout << "category: " << cate_pano.first << std::endl;        
            
            std::vector<float> res (3);
            cate_pano.second.compute_eval (res);
            float PQ = res[0];
            float SQ = res[1];
            float RQ = res[2];

            output_json["average"]["pano"][cate_pano.first] = { {"PQ", PQ * 100}, {"RQ", RQ * 100}, {"SQ", SQ * 100} };

            mPQ += PQ;
            mRQ += RQ;
            mSQ += SQ;

            std::cout << "PQ: " << PQ * 100 <<std::endl;
            std::cout << "SQ: " << SQ * 100 <<std::endl;
            std::cout << "RQ: " << RQ * 100 <<std::endl;
        }

        // Class-averaged result
        mPQ /= (float)all_pano_matches.size();
        mRQ /= (float)all_pano_matches.size();
        mSQ /= (float)all_pano_matches.size();
        output_json["average"]["pano"]["average"] = { {"mPQ", mPQ * 100}, {"mRQ", mRQ * 100}, {"mSQ", mSQ * 100} };

        std::cout << "mPQ: " << mPQ * 100 <<std::endl;
        std::cout << "mSQ: " << mSQ * 100 <<std::endl;
        std::cout << "mRQ: " << mRQ * 100 <<std::endl;
        


        /**************** 3D instance segmentation **************/
        std::cout << "------- 3D instance segmentation -------" << std::endl;

        float mAP = 0.0f;
        for (auto& cate_inst: all_inst_matches)
        {
            float current_ap = cate_inst.second.compute_AP();
            output_json["average"]["inst"][cate_inst.first] = { {"AP", current_ap * 100} };

            mAP += current_ap;

            std::cout << "category: " << cate_inst.first << std::endl;
            std::cout << "AP: " << current_ap * 100 <<std::endl;
        }
        mAP /= (float)all_inst_matches.size();
        output_json["average"]["inst"]["average"] = { {"mAP", mAP * 100} };

        std::cout << "mAP: " << mAP * 100 << std::endl;
        


        /**************** 3D box estimation **************/
        std::cout << "-------3D box estimation -------" << std::endl;

        float box_mAP = 0.0f;
        for (auto& cate_box: all_box_matches)
        {
            float current_ap = cate_box.second.compute_AP();
            output_json["average"]["bbox"][cate_box.first] = { {"AP", current_ap * 100} };

            box_mAP += current_ap;

            std::cout << "category: " << cate_box.first << std::endl;
            std::cout << "AP: " << current_ap * 100 <<std::endl;
        }
        box_mAP /= (float)all_box_matches.size();
        output_json["average"]["bbox"]["average"] = { {"mAP", box_mAP * 100} };

        std::cout << "mAP: " << box_mAP * 100 << std::endl;
    }

    std::string s = output_json.dump(4);
    std::ofstream out(output_folder_ + "/" + save_result_file_name_);
    if(out.is_open())
    {
        out << s;
        out.close();
    }
    else
        ROS_ERROR("Cannot open file: ", output_folder_ + "/" + save_result_file_name_);

    std::cout << "SUCCESS!" << std::endl;

}

}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_processing");
    ros::start();

    ros::NodeHandle node_handle("~"); // resolve namespace of node handle
    MapProcessing::EvalNode eval_node (node_handle);

    eval_node.Run();

    ros::spin(); // spin() will not return until the node has been shutdown
    ros::shutdown();

    return 0;
}
