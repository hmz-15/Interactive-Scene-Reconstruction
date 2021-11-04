#include "common.h"
#include "io.h"
#include "visualization.h"
#include <tinyxml.h>

namespace MapProcessing
{


void ExtractSegFromGTMap(const std::string current_GT_annotation_path, const std::string sequence_id)
{
    pcl::PointCloud<PointTLabel>::Ptr GT_map_cloud (new pcl::PointCloud<PointTLabel>);
    pcl::io::loadPLYFile(current_GT_annotation_path + sequence_id + "/" + sequence_id + ".ply", *GT_map_cloud);

    Viewer::Ptr viewer = std::make_shared<Viewer> (); 

    TiXmlDocument doc(current_GT_annotation_path + sequence_id + "/" + sequence_id + ".xml");
    bool loadOkay = doc.LoadFile();
	if (loadOkay)
    {
        TiXmlElement *annotation = doc.FirstChildElement("annotation");
        // TiXmlNode *label = annotation->FirstChild("label");
        TiXmlNode *label = 0;
        while (label = annotation->IterateChildren(label))
        {
            std::string str = label->ToElement()->Attribute("id");         
            int id = std::stoi(str);

            std::string text_str = label->ToElement()->Attribute("text");
            if (text_str.size() == 0)
                continue;

            std::cout << id << " " << text_str << std::endl;

            pcl::PointCloud<PointTLabel>::Ptr label_cloud (new pcl::PointCloud<PointTLabel>);
            ExtractCloudByLabel (GT_map_cloud, label_cloud, id);
            // std::cout << label_cloud->points.size() << std::endl;
            pcl::PointCloud<PointTFull>::Ptr full_cloud (new pcl::PointCloud<PointTFull>);
            pcl::copyPointCloud(*label_cloud, *full_cloud);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*label_cloud, *color_cloud);

            viewer->AddPointClouds ({color_cloud});
            viewer->VisualizeOnce();
        }
    }
}

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "extract_seg");
    ros::start();

    std::vector<std::string> sequence_id_list_;
    std::string GT_annotation_path_;

    ros::NodeHandle node_handle("~"); // resolve namespace of node handle
    node_handle.param<std::vector<std::string>>("sequence_id", sequence_id_list_, {});
    node_handle.param<std::string>("GT_annotation_path", GT_annotation_path_, "");

    for (auto& sequence_id: sequence_id_list_)
    {
        std::cout << sequence_id << std::endl;
        MapProcessing::ExtractSegFromGTMap(GT_annotation_path_, sequence_id);
    }

    ros::shutdown();

    return 0;
}