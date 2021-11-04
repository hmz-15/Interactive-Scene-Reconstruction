#include "pg_map_ros/object_node.h"


using namespace std;

namespace pgm{

ostream & operator<<(ostream &os, const ObjectNode &node)
{
    os << "[" << node.node_type_ << "]\n";
    os << "\tID:\t" << node.id_ << "\n";
    os << "\tLabel:\t" << node.label_ << "\n";
    os << "\tCAD ID:\t" << node.cad_id_ << "\n";
    os << "\tScale:\t" << node.bbox_ << "\n";
    os << "\tPosition:\t" << node.position_ << "\n";
    os << "\tOrientation:\t" << node.orientation_ << "\n";
    os << "\tBoundBox:\t" << node.bbox_ << "\t(l, w, h)\n";
    os << "\tnChild:\t" << node.children_.size() << "\n";
    
    os << "\tIoUs:\n";
    for(const auto &it : node.ious_){
        os << "\t\tlabel: " << it.first << ", IoU: " << it.second << "\n";
    }

    return os;
}


ostream & ObjectNode::output(ostream &os) const
{
    os << "Addr.: " << this << ", ";
    os << *this;
    return os;
}


ObjectNode::ObjectNode(
    int id, 
    const string &label,
    const string &cad_id,
    float scale,
    const Point &pos,
    const Quaternion &orient,
    const Point &box,
    const VecPair<int, float> &ious=VecPair<int, float>({{-1, 1.0}})
):
    NodeBase(NodeType::ObjectNode, id)
{
    label_ = label;
    cad_id_ = cad_id;
    scale_ = scale;
    setPose(pos, orient);
    setBBox(box);
    setIoUs(ious);
}


ObjectNode::ObjectNode(int id, const string label):
    NodeBase(NodeType::ObjectNode, id)
{
    label_ = label;
    cad_id_ = "";
    scale_ = 1.0;
    setPose(Point(0, 0, 0), Quaternion(0, 0, 0, 1));
    setBBox(Point(1, 1, 1));
    setIoUs(VecPair<int, float>( {make_pair(-1, 1.0)} ));
}


/*******************************************************************
 * Element Access
 ******************************************************************/

/**
 * Get the label of the node
 * 
 * @return the label of the node
 */
string ObjectNode::getLabel()
{
    return label_;
}


/**
 * Get the CAD ID of the node
 * 
 * @return the cad ID (std::string) of the node
 */
string ObjectNode::getCadID()
{
    return cad_id_;
}


/**
 * Get the scale of the object
 * 
 * @return the scale (std::string) of the object
 */
float ObjectNode::getScale()
{
    return scale_;
}


/**
 * Get the position of the object
 * 
 * @return the position (pgm::Point) of the object
 */
Point ObjectNode::getPosition()
{
    return position_;
}


/**
 * Get the orientation of the node
 * 
 * @return the orientation (pgm::Quaternion) of the node
 */
Quaternion ObjectNode::getOrientation()
{
    return orientation_;
}


/**
 * Get the bounding box of the object
 * 
 * The bounding box specifies the size (in 3D) of the object
 * 
 * @return the bounding box (pgm::Point) of the object
 */
Point ObjectNode::getBBox()
{
    return bbox_;
}

/**
 * Get the IoU of the replaced CAD and groud-truth segmented meshes
 * 
 * The IoU (Intersection over Union) of the replaced CAD and segmented meshes
 * Noted that, the replaced CAD could be overlaped with multiple groud-truth
 * segmented meshes.
 * 
 * @return the IoUs (a vector of pair<int, float>) of the replaced CAD 
 *         and segmented meshes
 */
VecPair<int, float> ObjectNode::getIoUs()
{
    return ious_;
}


/*******************************************************************
 * Modifier 
 ******************************************************************/

/**
 * Set the label of the node
 * 
 * @param label the label to be set 
 */
void ObjectNode::setLabel(const string &label)
{
    label_ = label;
}


/**
 * Set the CAD ID of the node
 * 
 * @param cad_id the cad ID to be set 
 */
void ObjectNode::setCadID(const std::string &cad_id)
{
    cad_id_ = cad_id;
}


/**
 * Set the scale of the object
 * 
 * @param scale the scale to be set 
 */
void ObjectNode::setScale(const float &scale)
{
    scale_ = scale;
}


/**
 * Set the pose of the object
 * 
 * @param pos the Position of the object
 * @param orient the orientation of the object
 */
void ObjectNode::setPose(const Point &pos, const Quaternion &orient)
{
    setPosition(pos);
    setOrientation(orient);
}


/**
 * Set the position of the object
 * 
 * @param pos the Position of the object
 */
void ObjectNode::setPosition(const Point &pos)
{
    position_.x = pos.x;
    position_.y = pos.y;
    position_.z = pos.z;
}


/**
 * Set the orientation of the object
 * 
 * @param orient the orientation of the object
 */
void ObjectNode::setOrientation(const Quaternion &orient)
{
    orientation_.x = orient.x;
    orientation_.y = orient.y;
    orientation_.z = orient.z;
    orientation_.w = orient.w;    
}


/**
 * Set the bounding box of the object
 * 
 * The bounding box specifies the size (in 3D) of the object
 * 
 * @param box the bounding box of the object
 */
void ObjectNode::setBBox(const Point &box)
{
    bbox_.x = box.x;
    bbox_.y = box.y;
    bbox_.z = box.z;
}


/**
 * Set the IoUs of the object
 * 
 * The IoU (Intersection over Union) of the replaced CAD and segmented meshes
 * 
 * Noted that, the replaced CAD could be overlaped with multiple groud-truth
 * segmented meshes. 
 * 
 * @param ious (VecPair<int, float>) a vector of pair<int, float> in a form of
 *        (label, iou_score) pair.
 */
void ObjectNode::setIoUs(const VecPair<int, float> &ious)
{
    ious_ = ious;
}


/**
 * Add the multiple IoU pairs to the object
 * 
 * Add (append) multiple IoUs (Intersection over Union) of the replaced CAD and 
 * segmented meshes.
 * 
 * Noted that, the replaced CAD could be overlaped with multiple groud-truth
 * segmented meshes. 
 * 
 * @param ious (VecPair<int, float>) a vector of pair<int, float> in a form of
 *        (label, iou_score) pair.
 */
void ObjectNode::addIoUs(const VecPair<int, float> &vp){
    for(const auto &it : vp){
        addIoU(it);
    }
}


/**
 * Add a single IoU pair to the object
 * 
 * Add (append) a single IoU (Intersection over Union) of the replaced CAD and 
 * segmented meshes.
 * 
 * Noted that, the replaced CAD could be overlaped with multiple groud-truth
 * segmented meshes. 
 * 
 * @param iou (std::pair<int, float>) pair<int, float> in a form of (label, iou_score).
 */
inline void ObjectNode::addIoU(const std::pair<int, float> &iou){
    ious_.emplace_back(iou);
}


/**
 * Add a single IoU pair to the object
 * 
 * Add (append) a single IoU (Intersection over Union) of the replaced CAD and 
 * segmented meshes.
 * 
 * Noted that, the replaced CAD could be overlaped with multiple groud-truth
 * segmented meshes. 
 * 
 * @param label (int) the label ID of the groud-truth segmented mesh
 * @param iou_score (float) the IoU score
 */
void ObjectNode::addIoU(int label, float iou_score){
    addIoU( make_pair(label, iou_score) );
}


} // End of pgm namespace


// ostream & operator<<(ostream &os, const ObjectNode::Ptr pnode)
// {
//   os << "Hi there, I am a " << pnode->node_type_ << ", I am a " << pnode->label_;
//   os << ", and my id is " << pnode->id_ << ".";

//   return os;
// }