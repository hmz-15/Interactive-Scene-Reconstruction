#ifndef EVAL_NODE_H_
#define EVAL_NODE_H_

#include "common.h"
#include "visualization.h"
#include "io.h"
#include "utils.h"
#include "object_cad.h"

#include <tinyxml.h>
#include "3rd_party/json.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace MapProcessing
{
class PanoInfo
{

public:
    PanoInfo(float iou, float TP, float FP, float FN): iou_sum(iou), TP_sum(TP), FP_sum(FP), FN_sum(FN) {} // Initialize FP. FN as # of predictions / gt

    float get_iou() const
    {
        return iou_sum;
    }

    void get_TP_FP_FN(std::vector<float>& results) const
    {
        if (results.size() < 3)
            results.reserve(3);
        results[0] = TP_sum;
        results[1] = FP_sum;
        results[2] = FN_sum;
    }

    void add(const PanoInfo& new_info)
    {
        iou_sum += new_info.get_iou();
        std::vector<float> res (3);
        new_info.get_TP_FP_FN(res);
        TP_sum += res[0];
        FP_sum += res[1];
        FN_sum += res[2];
    }

    void add_prediction(float iou)
    {
        iou_sum += iou;
        TP_sum += 1.0f;
        FP_sum -= 1.0f;
        FN_sum -= 1.0f;
    }

    // Compute PQ, SQ, RQ
    void compute_eval (std::vector<float>& results) const
    {
        if (results.size() < 3)
            results.reserve(3);
        if (TP_sum == 0.0f)
        {
            results[1] = 0.0f;
            results[2] = 0.0f;
            results[0] = 0.0f;
        }
        else
        {
            results[1] = iou_sum / TP_sum;   // SQ
            results[2] = TP_sum / (TP_sum + FP_sum/2 + FN_sum/2);  //RQ
            results[0] = results[1] * results[2];  //PQ
        } 
    }
    
private:
    float iou_sum = 0.0f;
    float TP_sum = 0.0f, FP_sum = 0.0f, FN_sum = 0.0f;    
};


class InstInfo
{
public:
    InstInfo(float GT_num): GT_size(GT_num)  {}

    float get_GT_size() const
    {
        return GT_size;
    }

    std::vector<std::pair<float, bool>> get_predictions() const
    {
        std::vector<std::pair<float, bool>> results (predictions.begin(), predictions.end());
        return results;
    }

    void add(const InstInfo& new_info)
    {
        GT_size += new_info.get_GT_size();
        std::vector<std::pair<float, bool>> new_pred = new_info.get_predictions();
        predictions.insert(predictions.end(), new_pred.begin(), new_pred.end());
    }

    void add_prediction(float confidence, bool success)
    {
        predictions.push_back(std::make_pair(confidence, success));
    }

    void sort_predictions()
    {
        auto cmp = [](const auto& a, const auto& b) -> bool {return a.first > b.first;};
        std::sort(predictions.begin(), predictions.end(), cmp);
    }

    float compute_AP() 
    {
        sort_predictions();
        // All false detection
        if (predictions.size() == 0)
            return 0.0f;

        // Compute PR curve
        std::vector<float> precision;
        std::vector<float> recall;
        float correct = 0.0f;
        float all_prediction = 0.0f;
        for (const auto& prediction: predictions)
        {
            all_prediction += 1.0f;
            if (prediction.second)
                correct += 1.0f;
            
            float current_precision = correct/all_prediction;
            float current_recall = correct/GT_size;
            precision.push_back(current_precision);
            recall.push_back(current_recall);
        }

        // Compute AP
        float p_sum = 0.0f;
        float step = 0.1;
        for (int rec_index = 0; rec_index < 11; rec_index ++)
        {
            float rec = (float)rec_index * step;
            if (rec >= recall.back())
            {
                p_sum += precision.back();
                continue;
            }

            for (int index = 0; index < recall.size(); index++)
            {
                if (recall[index] >= rec)
                {
                    std::vector<float> current_precision;
                    current_precision.insert(current_precision.end(), precision.begin()+index, precision.end());
                    std::sort(current_precision.begin(), current_precision.end());
                    p_sum += current_precision.back();
                    break;
                }
            }
        }
        p_sum /= float(11);
        return p_sum;
    }


private:
    std::vector<std::pair<float, bool>> predictions = {}; // confidence & correct
    float GT_size = 0.0f;

};


class EvalNode
{
public:
    EvalNode(ros::NodeHandle& node_handle);
    ~EvalNode(){};
    void LoadPanopticMap(const std::string sequence_id, std::map<std::string, std::vector<Obj3D::Ptr>>& objects, pcl::PointCloud<PointTLabel>::Ptr& map_cloud);
    void ProcessGTMap(const std::string sequence_id, std::map<std::string, std::vector<Obj3D::Ptr>>& objects, pcl::PointCloud<PointTLabel>::Ptr& map_cloud);
    void ComputeMatch (const std::map<std::string, std::vector<Obj3D::Ptr>>& map_objects, const std::map<std::string, std::vector<Obj3D::Ptr>>& GT_objects, 
                            const pcl::PointCloud<PointTLabel>::Ptr& map_cloud, std::unordered_map<std::string, PanoInfo>& pano_match, 
                            std::unordered_map<std::string, InstInfo>& inst_match, std::unordered_map<std::string, InstInfo>& box_match);

    nlohmann::json RunOnce(const std::string sequence_id, std::unordered_map<std::string, PanoInfo>& all_pano_matches, 
                                std::unordered_map<std::string, InstInfo>& all_inst_matches,  std::unordered_map<std::string, InstInfo>& all_box_matches);
    void Run();
    

private:
    ros::NodeHandle node_handle_;

    std::vector<std::string> sequence_id_list_;
    std::string output_folder_;
    std::string GT_annotation_path_;
    std::string save_result_file_name_;    

    Viewer::Ptr viewer;
    bool if_verbose_;
    bool use_nyu_class_;
    bool use_refined_id_;
    bool compute_per_class_eval_;


};

}





#endif