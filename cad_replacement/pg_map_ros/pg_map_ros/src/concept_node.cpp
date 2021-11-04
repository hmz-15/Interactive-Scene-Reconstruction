#include "pg_map_ros/concept_node.h"


using namespace std;

namespace pgm{

// overwrite the output function of the class
ostream & operator<<(ostream &os, const ConceptNode &node)
{
    os << "[" << node.node_type_ << "]\n";
    os << "\tID: " << node.id_ << "\n";
    os << "\tConcept: " << node.concept_ << "\n";
    os << "\tnChild: " << node.children_.size();

    return os;
}


// Overwrite the pure virtual function from NodeBase
ostream & ConceptNode::output(ostream &os) const
{
    os << "Addr.: " << this << ", ";
    os << *this;
    return os;
}


ConceptNode::ConceptNode(int id, string concept=""):
    NodeBase(NodeType::ConceptNode, id)
{
    concept_ = concept;
}


/*******************************************************************
 * Element Access
 ******************************************************************/

/**
 * Validate current parse graph
 * 
 * Check if node_dict edge_set are correct. This method could
 * be time-consuming.
 * 
 * @return true if valid, otherwise false
 */
string ConceptNode::getConcept()
{
    return concept_;
}


/*******************************************************************
 * Modifier 
 ******************************************************************/
void ConceptNode::setConcept(const string &concept)
{
    concept_ = concept;
}

} // End of pgm namespace


// ostream & operator<<(ostream &os, const ConceptNode::Ptr pnode)
// {
//   os << "Hi there, I am a " << pnode->node_type_ << ", my concept is " << pnode->concept_;
//   os << ", and my id is " << pnode->id_ << ".";

//   return os;
// }