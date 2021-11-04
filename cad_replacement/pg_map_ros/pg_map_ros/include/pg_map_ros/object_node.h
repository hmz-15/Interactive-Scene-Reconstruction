#ifndef _OBJECT_NODE_H
#define _OBJECT_NODE_H

#include "node_type.h"
#include "node_base.h"

#include <string>
#include <memory>
#include <iostream>


namespace pgm{

class ObjectNode: public NodeBase
{
public:
    using Ptr = std::shared_ptr<ObjectNode>;

    /**
     * Constructor
     * 
     * @param label the label of the node
     */
    ObjectNode(int, const std::string="");


    /**
     * Constructor
     * 
     * @param label (std::string) the label of the node
     * @param cad_id (std::string) the id of cad model
     * @param scale (float) the scale of the object comparing to the CAD model
     * @param pos (Point) the position of the object
     * @param orient (Quaternion) the orientation of the object
     * @param bbox (Point) the bounding box of the object
     * @param iou (float) the IoU of replaced CAD comparing to the segmented mesh
     */
    ObjectNode(int, const std::string &, const std::string &, float, const Point&, const Quaternion&, const Point&, const VecPair<int, float>&);

    ~ObjectNode() = default;

    friend std::ostream& operator<<(std::ostream&, const ObjectNode&);

    // friend std::ostream& operator<<(std::ostream&, const ObjectNode::Ptr);


/*******************************************************************
 * Element Access
 ******************************************************************/
public:
    /**
     * Get the label of the node
     * 
     * @return the label (std::string) of the node
     */
    std::string getLabel();


    /**
     * Get the CAD ID of the node
     * 
     * @return the cad ID (std::string) of the node
     */
    std::string getCadID();

    
    /**
     * Get the scale of the object
     * 
     * @return the scale (float) of the object
     */
    float getScale();
    

    /**
     * Get the position of the object
     * 
     * @return the position (pgm::Point) of the object
     */
    Point getPosition();


    /**
     * Get the orientation of the object
     * 
     * @return the orientation (pgm::Quaternion) of the object
     */
    Quaternion getOrientation();


    /**
     * Get the bounding box of the object
     * 
     * The bounding box specifies the size (in 3D) of the object
     * 
     * @return the bounding box (pgm::Point) of the object
     */
    Point getBBox();


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
    VecPair<int, float> getIoUs();

/*******************************************************************
 * Modifier 
 ******************************************************************/
public:
    /**
     * Set the label of the node
     * 
     * @param label the label to be set 
     */
    void setLabel(const std::string &);

    
    /**
     * Set the CAD ID of the node
     * 
     * @param cad_id the cad ID to be set 
     */
    void setCadID(const std::string &);


    /**
     * Set the scale of the object
     * 
     * @param scale the scale to be set
     */
    void setScale(const float &);


    /**
     * Set the pose of the object
     * 
     * @param pos the Position of the object
     * @param orient the orientation of the object
     */
    void setPose(const Point &, const Quaternion &);


    /**
     * Set the position of the object
     * 
     * @param pos the Position of the object
     */
    void setPosition(const Point &);
    

    /**
     * Set the orientation of the object
     * 
     * @param orient the orientation of the object
     */
    void setOrientation(const Quaternion &);


    /**
     * Set the bounding box of the object
     * 
     * The bounding box specifies the size (in 3D) of the object
     * 
     * @param box the bounding box of the object
     */
    void setBBox(const Point &);

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
    void setIoUs(const VecPair<int, float>&);


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
    void addIoUs(const VecPair<int, float>&);


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
    inline void addIoU(const std::pair<int, float>&);


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
    void addIoU(int, float);


protected:
    virtual std::ostream & output(std::ostream &) const;

private:
    std::string label_;
    std::string cad_id_;
    float scale_;
    Point position_;
    Quaternion orientation_;
    Point bbox_;
    VecPair<int, float> ious_;
};

} // end of namespace pgm

#endif