#ifndef _PG_EDGE_H
#define _PG_EDGE_H

#include <iostream>
#include <utility>

namespace pgm
{

class PgEdge
{
public:
    // PgEdge();
    PgEdge(int, int);

    friend std::ostream & operator << (std::ostream &, const PgEdge&);


/*******************************************************************
 * Element Access
 ******************************************************************/
public:
    /**
     * Get source node ID
     * 
     * @return ID of the source node
     */
    int getSrcID() const;

    /**
     * Get destination node ID
     * 
     * @return ID of the destination node
     */
    int getDstID() const;
    
private:
    int src_id_;
    int dst_id_;
};

}   // end of namespace pgm

#endif