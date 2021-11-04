#include "pg_map_ros/node_base.h"


using namespace std;


namespace pgm{


NodeBase::NodeBase(NodeType node_type, int id)
{
    node_type_ = node_type;
    id_ = id;
}


ostream & operator << (ostream &os, const NodeBase &node)
{
    return node.output(os);
}


ostream & operator << (ostream &os, const NodeBase::Ptr pnode)
{
    return pnode->output(os);
}


/*******************************************************************
 * Element Access
 ******************************************************************/

/**
 * Get type of the node
 * 
 * @return node type defined by enum class NodeType
 */
NodeType NodeBase::getNodeType()
{
    return node_type_;
}


/**
 * Get the ID of node
 * 
 * @return the ID of the node
 */
int NodeBase::getID()
{
    return id_;
}


/**
 * Get the number of children
 * 
 * Get the number of direct children of current node
 * 
 * @return the number of children
 */
int NodeBase::childrenCount()
{
    return children_.size();
}


/**
 * Get the list of children
 * 
 * Get the list of children of the node
 * 
 * @return A vector of pointers of the children
 */
vector<NodeBase::Ptr> NodeBase::getChildren()
{
    return children_;
}


/*******************************************************************
 * Modifiers
 ******************************************************************/

/**
 * Add a child to the node
 * 
 * @param child the pointer of the child to be added
 */
void NodeBase::addChild(NodeBase::Ptr child)
{
    children_.push_back(child);
}


/**
 * Delete a child from the node
 * 
 * Delete a node from the children_ list of the node
 * 
 * @param pnode the pointer of the node to be added
 */
void NodeBase::deleteChild(NodeBase::Ptr pnode)
{
    int idx = -1;
    for(size_t i = 0; i < children_.size(); i++)
    {
        if(children_[i] == pnode)
        {
            idx = i;
            break;
        }
    }

    if(idx != - 1)
    {
        children_.erase(children_.begin() + idx);
    }
}

} // end of namespace pgm