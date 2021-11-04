#include <iostream>

#include "pg_map_ros/node_type.h"

using namespace std;

namespace pgm
{

ostream & operator<<(ostream &os, const NodeType node_t)
{
    switch (node_t)
    {
    case NodeType::ObjectNode:
        os << "ObjectNode";
        break;

    case NodeType::ConceptNode:
        os << "ConceptNode";
        break;
    
    default:
        break;
    }

    return os;
}


std::ostream &operator<<(std::ostream& os, const Quaternion quat)
{
    os << "(" << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << ")";

    return os;
}


std::ostream &operator<<(std::ostream& os, const Point pt)
{
    os << "(" << pt.x << ", " << pt.y << ", " << pt.z << ")";

    return os;
}


} // end of namespace pgm
