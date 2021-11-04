#include "pg_map_ros/pg_edge.h"


using namespace std;


namespace pgm
{

ostream & operator << (ostream &os, const PgEdge &e)
{
    os << e.src_id_ << " --> " << e.dst_id_;
    return os;
}

// PgEdge::PgEdge()
// {
//     src_id_ = -1;
//     dst_id_ = -1;
// }

PgEdge::PgEdge(int src_id, int dst_id)
{
    src_id_ = src_id;
    dst_id_ = dst_id;
}

/**
 * Get source node ID
 * 
 * @return ID of the source node
 */
int PgEdge::getSrcID() const
{
    return src_id_;
}

/**
 * Get destination node ID
 * 
 * @return ID of the destination node
 */
int PgEdge::getDstID() const
{
    return dst_id_;
}

} // end of namespace pgm