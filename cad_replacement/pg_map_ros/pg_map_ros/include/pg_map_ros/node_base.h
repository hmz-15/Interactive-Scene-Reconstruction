#ifndef _NODE_BASE_H
#define _NODE_BASE_H

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>

#include "node_type.h"

namespace pgm{

class NodeBase
{
public:
    using Ptr = std::shared_ptr<NodeBase>;

    NodeBase(NodeType node_type, int id);

    virtual ~NodeBase() = default;

    friend std::ostream & operator << (std::ostream &, const NodeBase &);

    friend std::ostream & operator << (std::ostream &, const NodeBase::Ptr);


/*******************************************************************
 * Element Access
 ******************************************************************/
public:
    /**
     * Get type of the node
     * 
     * @return node type defined by enum class NodeType
     */
    NodeType getNodeType();


    /**
     * Get the ID of node
     * 
     * @return the ID of the node
     */
    int getID();


    /**
     * Get the number of children
     * 
     * Get the number of direct children of current node
     * 
     * @return the number of children
     */
    int childrenCount();


    /**
     * Get the list of children
     * 
     * Get the list of children of the node
     * 
     * @return A vector of pointers of the children
     */
    std::vector<NodeBase::Ptr> getChildren();


/*******************************************************************
 * Modifiers
 ******************************************************************/
public:
    /**
     * Add a child to the node
     * 
     * @param child the pointer of the child to be added
     */
    void addChild(NodeBase::Ptr);
    

    /**
     * Delete a child from the node
     * 
     * Delete a node from the children_ list of the node
     * 
     * @param pnode the pointer of the node to be added
     */
    void deleteChild(NodeBase::Ptr);


protected:
    virtual std::ostream & output(std::ostream &) const = 0;

protected:
    int id_;
    NodeType node_type_;
    std::vector<NodeBase::Ptr> children_;
};

} // end of namespace pgm

#endif